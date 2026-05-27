"""End-to-end SIL braking test — Week 2 milestone.

The real co-simulation: plant Python advances the vehicle dynamics, the C
ECU process drives the brake. Both speak the binary protocol at 100 Hz.

Acceptance criteria (PHYSICS.md §5 + plan §S2):
    * stopping distance with the C ABS ≈ same as Python oracle (~45 m on dry)
    * the run completes (no comm timeout) over the full braking event
    * the C ECU reaches state ACTIVE at some point during the run

Usage (two terminals):
    # T1
    python -m scripts.sil_brake_test --duration 6 --surface dry_asphalt
    # T2 (start within 5 s — the server waits)
    ./controller/build/abs_ecu 127.0.0.1 9000 6 100

A `--spawn-ecu` flag will be added once we have a portable launcher; for now
running each side in its own WSL terminal is the canonical workflow.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from plant import protocol as P
from plant.socket_server import PlantLink, now_ms
from plant.tire import PacejkaTire
from plant.vehicle import Vehicle, VehicleParams


RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"

CONTROL_PERIOD_S = 0.010      # 10 ms, matches the ECU loop
PLANT_DT_S       = 0.001      # 1 ms integration step
STEPS_PER_TICK   = int(round(CONTROL_PERIOD_S / PLANT_DT_S))


# --- Stable list of state-name strings (matches abs_state_name in C) -------

STATE_NAMES = {
    P.ECU_INIT: "INIT", P.ECU_STANDBY: "STANDBY",
    P.ECU_MONITOR: "MONITOR", P.ECU_ACTIVE: "ACTIVE",
    P.ECU_FAULT_DEGRADED: "FAULT_DEGRADED",
    P.ECU_FAULT_LATCHED: "FAULT_LATCHED",
}


@dataclass
class SilRun:
    surface: str
    duration: float
    t: list[float]
    v: list[float]
    omega: list[float]
    slip: list[float]
    brake_cmd: list[float]
    state: list[int]
    dtc: list[int]
    x_stop: float
    t_stop: float
    saw_active: bool

    def to_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "v", "omega", "slip", "brake_cmd", "state", "dtc"])
            for row in zip(self.t, self.v, self.omega, self.slip,
                            self.brake_cmd, self.state, self.dtc):
                w.writerow(row)


def run(surface: str, duration: float, host: str, port: int,
        v0_mps: float, accept_timeout: float) -> int:
    veh = Vehicle(VehicleParams(), PacejkaTire(surface), v0_mps)
    link = PlantLink(host, port)
    link.listen()
    print(f"[sil] plant ready on {host}:{port}, waiting for ECU "
          f"(timeout {accept_timeout:.0f} s)...")
    try:
        link.accept(timeout=accept_timeout)
    except Exception as e:
        print(f"[sil] ECU did not connect: {e}", file=sys.stderr)
        return 2
    print("[sil] ECU connected, starting co-simulation.")

    log = SilRun(surface=surface, duration=duration,
                 t=[], v=[], omega=[], slip=[], brake_cmd=[],
                 state=[], dtc=[], x_stop=0.0, t_stop=0.0,
                 saw_active=False)

    n_ticks = int(round(duration / CONTROL_PERIOD_S))
    last_brake = 0.0
    saw_active = False

    for k in range(n_ticks):
        # 1. Compute slip on the *previous* step's state — same causal
        # ordering as if the wheel-speed sensor was read at the period start.
        slip = 0.0
        if veh.state.v > 1.0:
            slip = max(0.0, min(1.0,
                (veh.state.v - veh.state.omega * veh.params.wheel_radius)
                / veh.state.v))

        # 2. Ship the sensor frame to the ECU.
        s = P.SensorFrame(
            timestamp_ms=now_ms(),
            v_vehicle=veh.state.v,
            omega_wheel=veh.state.omega,
            brake_pressure=last_brake,
            fault_flags=0,
        )
        try:
            link.send_sensor(s)
        except (BrokenPipeError, ConnectionError) as e:
            print(f"[sil] link broken at tick {k}: {e}", file=sys.stderr)
            return 3

        # 3. Receive its decision. 50 ms watchdog matches the ECU side.
        a = link.recv_actuator(timeout=0.050)
        if a is None:
            print(f"[sil] FAIL: timeout waiting for ECU at tick {k} "
                  f"(t={k*CONTROL_PERIOD_S:.3f}s)", file=sys.stderr)
            return 4

        brake_cmd = a.brake_command
        last_brake = brake_cmd
        if a.ecu_status == P.ECU_ACTIVE:
            saw_active = True

        # 4. Advance the plant by 10 sub-steps of 1 ms each.
        for _ in range(STEPS_PER_TICK):
            veh.step(brake_cmd, dt=PLANT_DT_S)

        log.t.append(veh.state.t)
        log.v.append(veh.state.v)
        log.omega.append(veh.state.omega)
        log.slip.append(slip)
        log.brake_cmd.append(brake_cmd)
        log.state.append(a.ecu_status)
        log.dtc.append(a.dtc_code)

        if veh.stopped:
            break

    log.x_stop = veh.state.x
    log.t_stop = veh.state.t
    log.saw_active = saw_active
    link.close()

    # ---- report --------------------------------------------------------
    print(f"\n=== SIL braking test — surface={surface} ===")
    print(f"v0                  : {v0_mps:.2f} m/s ({v0_mps*3.6:.1f} km/h)")
    print(f"stopping distance   : {log.x_stop:.2f} m")
    print(f"stopping time       : {log.t_stop:.2f} s")
    print(f"saw ECU_ACTIVE      : {log.saw_active}")
    state_set = set(log.state)
    print(f"states visited      : {sorted({STATE_NAMES.get(s, str(s)) for s in state_set})}")
    dtc_union = 0
    for d in log.dtc: dtc_union |= d
    print(f"DTC union           : 0x{dtc_union:04X}")

    csv_out = RESULTS_DIR / f"sil_run_{surface}.csv"
    log.to_csv(csv_out)
    print(f"per-tick log        : {csv_out}")

    # ---- assertions ----------------------------------------------------
    crit: list[str] = []
    if not log.saw_active:
        crit.append("ECU never reached state ACTIVE — controller did not engage")
    if dtc_union != 0:
        crit.append(f"DTC raised during run: 0x{dtc_union:04X}")
    if surface == "dry_asphalt":
        if not (35.0 <= log.x_stop <= 55.0):
            crit.append(f"stopping distance {log.x_stop:.1f} m outside [35, 55]")

    if crit:
        for c in crit: print(f"  FAIL: {c}", file=sys.stderr)
        return 1
    print("\n[sil] OK — Week-2 milestone passed.")
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--surface", default="dry_asphalt",
                   choices=["dry_asphalt", "wet_asphalt", "packed_snow", "ice"])
    p.add_argument("--duration", type=float, default=6.0,
                   help="max wall-clock seconds of simulation")
    p.add_argument("--v0", type=float, default=27.78,
                   help="initial speed in m/s (default 100 km/h)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--accept-timeout", type=float, default=15.0)
    args = p.parse_args(argv)
    return run(args.surface, args.duration, args.host, args.port,
               args.v0, args.accept_timeout)


if __name__ == "__main__":
    sys.exit(main())
