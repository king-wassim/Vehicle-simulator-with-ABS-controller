"""Offline simulation runner — week 1 / days 3-4.

Validates the plant model in isolation, before any sockets, by running two
canonical scenarios and producing comparison plots:

    1. no_abs   : driver mashes the pedal — wheel locks, ~60 m stopping distance.
    2. oracle   : Python bang-bang ABS in [0.10, 0.20] — ~45 m stopping distance.

The numbers above are the validation targets from PHYSICS.md §5.

Usage:
    python -m plant.simulate                       # both scenarios + plots
    python -m plant.simulate --scenario no_abs     # one scenario, prints stats
    python -m plant.simulate --no-plot             # CI-friendly, just stats
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from pathlib import Path

from .oracle_abs import OracleABS
from .vehicle import Vehicle, VehicleParams
from .tire import PacejkaTire


# --- Simulation configuration ----------------------------------------------

DT_PLANT = 0.001          # 1 ms — plant integration step
DT_CONTROL = 0.010        # 10 ms — controller period (matches future ECU)
SIM_DURATION = 6.0        # s — bail out if vehicle hasn't stopped by then
INITIAL_SPEED = 27.78     # m/s = 100 km/h
DRIVER_PEDAL_BAR = 100.0  # constant pedal force — "panic braking"

RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


# --- Log container ---------------------------------------------------------

@dataclass
class SimLog:
    """In-memory time series of one run — easier to plot than streaming CSV."""

    t: list[float]
    v: list[float]
    omega_R: list[float]    # omega * R, same units as v (m/s) for direct comparison
    slip: list[float]
    mu: list[float]
    brake_cmd: list[float]
    x: list[float]
    scenario: str

    def stopping_distance(self) -> float:
        return self.x[-1]

    def stopping_time(self) -> float:
        return self.t[-1]

    def mean_slip_during_braking(self) -> float:
        # Average over samples where the vehicle is actually moving (above
        # ~5 km/h, the speed at which the oracle hands over to the driver).
        # Filtering on brake_cmd instead would bias the mean upward by
        # discarding the release phases where slip is intentionally low.
        active = [s for s, v in zip(self.slip, self.v) if v > 1.4]
        return sum(active) / len(active) if active else 0.0

    def max_slip(self) -> float:
        return max(self.slip)

    def write_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "v", "omega_R", "slip", "mu", "brake_cmd", "x"])
            for row in zip(self.t, self.v, self.omega_R, self.slip,
                           self.mu, self.brake_cmd, self.x):
                w.writerow(row)


# --- Scenarios -------------------------------------------------------------

def run_no_abs(surface: str = "dry_asphalt") -> SimLog:
    """Driver mashes the pedal — no ABS modulation. Wheel locks early."""
    veh = Vehicle(VehicleParams(), PacejkaTire(surface), INITIAL_SPEED)
    return _run_loop(veh, controller=None, label="no_abs")


def run_oracle(surface: str = "dry_asphalt") -> SimLog:
    """Same panic braking, but the oracle ABS modulates the pressure."""
    veh = Vehicle(VehicleParams(), PacejkaTire(surface), INITIAL_SPEED)
    return _run_loop(veh, controller=OracleABS(), label="oracle")


def _run_loop(veh: Vehicle, controller: OracleABS | None, label: str) -> SimLog:
    log = SimLog([], [], [], [], [], [], [], scenario=label)

    steps_per_control = max(1, int(round(DT_CONTROL / DT_PLANT)))
    n_total = int(round(SIM_DURATION / DT_PLANT))

    # Latest control command — refreshed every steps_per_control plant steps.
    brake_cmd = 0.0

    for k in range(n_total):
        if veh.stopped:
            # Pad one final sample so logs are complete and then exit.
            log.t.append(veh.state.t)
            log.v.append(veh.state.v)
            log.omega_R.append(veh.state.omega * veh.params.wheel_radius)
            log.slip.append(0.0)
            log.mu.append(0.0)
            log.brake_cmd.append(brake_cmd)
            log.x.append(veh.state.x)
            break

        if k % steps_per_control == 0:
            # Compute control with the *previous* step's slip — same causal
            # ordering as a real ECU reading sensors at the start of its cycle.
            slip_for_ctrl = log.slip[-1] if log.slip else 0.0
            if controller is None:
                brake_cmd = DRIVER_PEDAL_BAR
            else:
                brake_cmd = controller.step(slip_for_ctrl, veh.state.v,
                                             driver_request=DRIVER_PEDAL_BAR)

        out = veh.step(brake_cmd, dt=DT_PLANT)

        log.t.append(veh.state.t)
        log.v.append(veh.state.v)
        log.omega_R.append(veh.state.omega * veh.params.wheel_radius)
        log.slip.append(out.slip)
        log.mu.append(out.mu)
        log.brake_cmd.append(out.brake_pressure)
        log.x.append(veh.state.x)

    return log


# --- Plotting --------------------------------------------------------------

def plot_logs(logs: list[SimLog], output: Path) -> None:
    """Three-panel summary: speeds, slip, brake command."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    colors = {"no_abs": "tab:red", "oracle": "tab:green"}

    for log in logs:
        c = colors.get(log.scenario, "tab:blue")
        axes[0].plot(log.t, log.v, label=f"{log.scenario} — v", color=c)
        axes[0].plot(log.t, log.omega_R, label=f"{log.scenario} — ω·R",
                     color=c, linestyle="--", alpha=0.7)
        axes[1].plot(log.t, log.slip, label=log.scenario, color=c)
        axes[2].plot(log.t, log.brake_cmd, label=log.scenario, color=c)

    axes[0].set_ylabel("speed (m/s)")
    axes[0].set_title("Vehicle and wheel speed")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    axes[1].set_ylabel("slip ratio")
    axes[1].set_title("Slip — target zone shaded")
    axes[1].axhspan(0.10, 0.20, color="gray", alpha=0.15, label="ABS target")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].set_ylabel("brake pressure (bar)")
    axes[2].set_xlabel("time (s)")
    axes[2].set_title("Brake command")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output, dpi=110)
    print(f"  saved plot -> {output}")


# --- CLI -------------------------------------------------------------------

def _print_summary(log: SimLog) -> None:
    print(f"  scenario           : {log.scenario}")
    print(f"  stopping distance  : {log.stopping_distance():.2f} m")
    print(f"  stopping time      : {log.stopping_time():.2f} s")
    print(f"  mean slip (braking): {log.mean_slip_during_braking():.3f}")
    print(f"  max  slip          : {log.max_slip():.3f}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline plant simulation")
    parser.add_argument("--scenario", choices=["no_abs", "oracle", "both"],
                        default="both")
    parser.add_argument("--surface", default="dry_asphalt")
    parser.add_argument("--no-plot", action="store_true",
                        help="skip matplotlib (useful for CI / smoke tests)")
    parser.add_argument("--csv-out", type=Path, default=None,
                        help="write per-scenario CSV logs into this directory")
    args = parser.parse_args(argv)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    if args.csv_out:
        os.makedirs(args.csv_out, exist_ok=True)

    logs: list[SimLog] = []
    if args.scenario in ("no_abs", "both"):
        print("[no_abs] running…")
        logs.append(run_no_abs(args.surface))
        _print_summary(logs[-1])
    if args.scenario in ("oracle", "both"):
        print("[oracle] running…")
        logs.append(run_oracle(args.surface))
        _print_summary(logs[-1])

    for log in logs:
        if args.csv_out:
            log.write_csv(args.csv_out / f"{log.scenario}_{args.surface}.csv")

    if not args.no_plot:
        try:
            out = RESULTS_DIR / f"plant_validation_{args.surface}.png"
            plot_logs(logs, out)
        except ImportError:
            print("  matplotlib not installed — skipping plot")

    # Sanity assertions on the canonical dry-asphalt comparison.
    if args.scenario == "both" and args.surface == "dry_asphalt":
        no_abs_d  = logs[0].stopping_distance()
        oracle_d  = logs[1].stopping_distance()
        gain_pct  = 100.0 * (no_abs_d - oracle_d) / no_abs_d
        print(f"\n  no_abs={no_abs_d:.1f} m, oracle={oracle_d:.1f} m, gain={gain_pct:.1f}%")
        # Expected envelope per PHYSICS.md §5.
        ok_no_abs = 50.0 <= no_abs_d <= 75.0
        ok_oracle = 35.0 <= oracle_d <= 55.0
        ok_gain   = gain_pct >= 10.0
        if not (ok_no_abs and ok_oracle and ok_gain):
            print("  WARNING: numbers outside the expected envelope — recheck model.")
            return 1
        print("  [OK] within expected envelope (PHYSICS.md section 5)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
