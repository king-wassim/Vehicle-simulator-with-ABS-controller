"""Scenario harness — runs the plant ↔ ECU loop with an injected fault list.

Each scenario script imports `run_scenario` with its own FaultInjector and
assertion callable, and gets back a ScenarioResult it can write to CSV/JSON.

Compared to scripts/sil_brake_test.py:
  - accepts a FaultInjector (sensor / wire / environment hooks)
  - records DTC bitfield evolution over time
  - extracts richer per-tick state so the assertions can be specific
  - no built-in pass/fail — the caller decides what "success" means
"""

from __future__ import annotations

import csv
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

from plant import protocol as P
from plant.fault_injector import FaultInjector
from plant.socket_server import PlantLink, now_ms
from plant.tire import PacejkaTire
from plant.vehicle import Vehicle, VehicleParams


RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"
CONTROL_PERIOD_S = 0.010
PLANT_DT_S       = 0.001
STEPS_PER_TICK   = int(round(CONTROL_PERIOD_S / PLANT_DT_S))


@dataclass
class ScenarioConfig:
    name: str
    surface: str = "dry_asphalt"
    duration_s: float = 6.0
    v0_mps: float = 27.78
    driver_request_bar: float = 100.0
    host: str = "127.0.0.1"
    port: int = 9000
    accept_timeout_s: float = 10.0


@dataclass
class ScenarioResult:
    config: ScenarioConfig
    t: list[float] = field(default_factory=list)
    v: list[float] = field(default_factory=list)
    omega: list[float] = field(default_factory=list)
    slip: list[float] = field(default_factory=list)
    brake_cmd: list[float] = field(default_factory=list)
    state: list[int] = field(default_factory=list)
    dtc: list[int] = field(default_factory=list)
    active_faults: list[list[str]] = field(default_factory=list)

    def stopping_distance(self) -> float:
        return self._last_pos

    def stopping_time(self) -> float:
        return self.t[-1] if self.t else 0.0

    def states_visited(self) -> set[int]:
        return set(self.state)

    def dtc_union(self) -> int:
        out = 0
        for d in self.dtc:
            out |= d
        return out

    def first_dtc_time(self, dtc_mask: int) -> Optional[float]:
        for t, d in zip(self.t, self.dtc):
            if d & dtc_mask:
                return t
        return None

    def first_state_time(self, target: int) -> Optional[float]:
        for t, s in zip(self.t, self.state):
            if s == target:
                return t
        return None

    def to_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "v", "omega", "slip", "brake_cmd", "state",
                        "dtc", "active_faults"])
            for row in zip(self.t, self.v, self.omega, self.slip,
                           self.brake_cmd, self.state, self.dtc,
                           self.active_faults):
                w.writerow([*row[:7], ";".join(row[7])])

    _last_pos: float = 0.0


def _slip(v: float, omega: float, r: float) -> float:
    if v < 1.0: return 0.0
    s = (v - omega * r) / v
    return max(0.0, min(1.0, s))


def run_scenario(cfg: ScenarioConfig, injector: FaultInjector) -> ScenarioResult:
    veh = Vehicle(VehicleParams(), PacejkaTire(cfg.surface), cfg.v0_mps)
    link = PlantLink(cfg.host, cfg.port)
    link.listen()
    print(f"[{cfg.name}] waiting for ECU on {cfg.host}:{cfg.port} "
          f"(timeout {cfg.accept_timeout_s:.0f}s)...")
    link.accept(timeout=cfg.accept_timeout_s)
    print(f"[{cfg.name}] ECU connected. Faults: "
          f"{[type(f).__name__ for f in injector.faults]}")

    result = ScenarioResult(config=cfg)
    n_ticks = int(round(cfg.duration_s / CONTROL_PERIOD_S))
    last_brake = 0.0

    for k in range(n_ticks):
        t_s = k * CONTROL_PERIOD_S

        # 1. Environment faults (ice patch swaps tire surface in-place).
        injector.apply_environment(veh.tire, t_s)

        # 2. Build the raw sensor frame from the plant truth.
        raw = P.SensorFrame(
            timestamp_ms=now_ms(),
            v_vehicle=veh.state.v,
            omega_wheel=veh.state.omega,
            brake_pressure=last_brake,
            fault_flags=0,
        )

        # 3. Apply signal-level faults.
        filtered = injector.filter_sensor(raw, t_s)

        # 4. Send (unless the injector dropped it).
        if filtered is not None:
            wire = P.encode_frame(P.TYPE_SENSOR, filtered.pack_payload())
            wire = injector.filter_wire(wire, t_s)
            try:
                # We bypass `link.send_sensor` to inject pre-encoded bytes.
                link._conn.sendall(wire)
                link.stats.sensor_frames_sent += 1
            except (BrokenPipeError, ConnectionError):
                print(f"[{cfg.name}] link broken at t={t_s:.3f}s", file=sys.stderr)
                break

        # 5. Receive ECU response — short watchdog so dropped frames surface fast.
        a = link.recv_actuator(timeout=0.050)
        if a is None:
            # Timeout — log a placeholder and continue (don't abort: this is
            # exactly what comm_loss scenarios want to observe).
            a = P.ActuatorFrame(now_ms(), last_brake, P.ECU_FAULT_DEGRADED,
                                P.DTC_COMM_TIMEOUT)
        else:
            last_brake = a.brake_command

        # 6. Advance the plant 10× 1 ms steps.
        for _ in range(STEPS_PER_TICK):
            veh.step(last_brake, dt=PLANT_DT_S)

        # 7. Log.
        result.t.append(veh.state.t)
        result.v.append(veh.state.v)
        result.omega.append(veh.state.omega)
        result.slip.append(_slip(veh.state.v, veh.state.omega,
                                 veh.params.wheel_radius))
        result.brake_cmd.append(a.brake_command)
        result.state.append(a.ecu_status)
        result.dtc.append(a.dtc_code)
        result.active_faults.append(injector.active_names(t_s))

        if veh.stopped:
            break

    result._last_pos = veh.state.x
    link.close()
    return result


# --- assertion helpers -----------------------------------------------------

STATE_NAMES = {
    P.ECU_INIT: "INIT", P.ECU_STANDBY: "STANDBY",
    P.ECU_MONITOR: "MONITOR", P.ECU_ACTIVE: "ACTIVE",
    P.ECU_FAULT_DEGRADED: "FAULT_DEGRADED",
    P.ECU_FAULT_LATCHED: "FAULT_LATCHED",
}

DTC_NAMES = {
    P.DTC_SENSOR_STUCK: "STUCK",
    P.DTC_SENSOR_NOISE: "NOISE",
    P.DTC_COMM_TIMEOUT: "COMM_TIMEOUT",
    P.DTC_COMM_CRC:     "COMM_CRC",
    P.DTC_SENSOR_RANGE: "SENSOR_RANGE",
    P.DTC_PLAUSIBILITY: "PLAUSIBILITY",
}


def print_summary(r: ScenarioResult) -> None:
    print(f"\n=== Scenario: {r.config.name} (surface={r.config.surface}) ===")
    print(f"  stopping distance : {r.stopping_distance():.2f} m")
    print(f"  stopping time     : {r.stopping_time():.2f} s")
    states = {STATE_NAMES.get(s, str(s)) for s in r.states_visited()}
    print(f"  states visited    : {sorted(states)}")
    union = r.dtc_union()
    dtc_labels = [name for bit, name in DTC_NAMES.items() if union & bit]
    print(f"  DTC union         : 0x{union:04X} ({', '.join(dtc_labels) or 'none'})")


def assert_all(asserts: list[tuple[bool, str]]) -> int:
    """Pretty-print PASS/FAIL list and return process exit code."""
    failed = 0
    for ok, msg in asserts:
        print(f"  {'PASS' if ok else 'FAIL'} — {msg}")
        if not ok: failed += 1
    if failed:
        print(f"\n[scenario] {failed} assertion(s) failed", file=sys.stderr)
        return 1
    print("\n[scenario] OK")
    return 0
