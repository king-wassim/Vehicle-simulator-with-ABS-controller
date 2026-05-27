"""Generate the comparison plots that ship with docs/RESULTS.md.

Reads the CSV traces produced by the scenarios (`results/scenarios/*.csv`)
and the Python-oracle reference, then renders three PNGs:

  - oracle_vs_c_sil.png    : speed / slip / brake_cmd, Python oracle vs C SIL
  - scenarios_summary.png  : stopping distances + state visited per scenario
  - dtc_timeline.png       : DTC bits raised over time across all scenarios

These plots are saved into `results/` so the README / RESULTS.md can embed
them with relative paths and so the CI uploads them as artifacts.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plant import protocol as P
from plant.oracle_abs import OracleABS
from plant.tire import PacejkaTire
from plant.vehicle import Vehicle, VehicleParams


ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results"
SCEN_DIR = RESULTS / "scenarios"


STATE_NAMES = {
    P.ECU_INIT: "INIT", P.ECU_STANDBY: "STANDBY",
    P.ECU_MONITOR: "MONITOR", P.ECU_ACTIVE: "ACTIVE",
    P.ECU_FAULT_DEGRADED: "FAULT_DEGRADED",
    P.ECU_FAULT_LATCHED: "FAULT_LATCHED",
}
DTC_NAMES = [
    (P.DTC_SENSOR_STUCK, "STUCK"),
    (P.DTC_SENSOR_NOISE, "NOISE"),
    (P.DTC_COMM_TIMEOUT, "COMM_TIMEOUT"),
    (P.DTC_COMM_CRC,     "COMM_CRC"),
    (P.DTC_SENSOR_RANGE, "SENSOR_RANGE"),
    (P.DTC_PLAUSIBILITY, "PLAUSIBILITY"),
]


# --- helpers ----------------------------------------------------------------

def load_csv(path: Path) -> dict[str, list]:
    cols: dict[str, list] = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            for k, v in row.items():
                cols.setdefault(k, []).append(v)
    # convert numeric columns
    for k in ("t", "v", "omega", "slip", "brake_cmd"):
        if k in cols:
            cols[k] = [float(x) for x in cols[k]]
    for k in ("state", "dtc"):
        if k in cols:
            cols[k] = [int(x) for x in cols[k]]
    return cols


def run_python_oracle(duration_s: float = 6.0, dt: float = 0.001) -> dict[str, list]:
    """Recompute the Python-oracle braking run for comparison."""
    veh = Vehicle(VehicleParams(), PacejkaTire("dry_asphalt"), 27.78)
    ctrl = OracleABS()
    step_per_ctrl = int(round(0.010 / dt))
    n = int(round(duration_s / dt))
    log: dict[str, list] = {"t": [], "v": [], "omega": [], "slip": [], "brake_cmd": []}
    cmd = 0.0
    for k in range(n):
        if veh.stopped:
            break
        if k % step_per_ctrl == 0:
            slip = log["slip"][-1] if log["slip"] else 0.0
            cmd = ctrl.step(slip, veh.state.v, driver_request=100.0)
        out = veh.step(cmd, dt=dt)
        log["t"].append(veh.state.t)
        log["v"].append(veh.state.v)
        log["omega"].append(veh.state.omega)
        log["slip"].append(out.slip)
        log["brake_cmd"].append(out.brake_pressure)
    return log


# --- plot 1: oracle vs C SIL ------------------------------------------------

def plot_oracle_vs_sil(out: Path) -> None:
    oracle = run_python_oracle()
    sil = load_csv(SCEN_DIR / "s1_nominal.csv")

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(oracle["t"], oracle["v"], label="oracle (Python)", color="tab:blue")
    axes[0].plot(sil["t"], sil["v"], label="SIL (C)", color="tab:orange")
    axes[0].set_ylabel("vehicle speed (m/s)")
    axes[0].set_title("Python oracle vs end-to-end C SIL — dry asphalt, panic brake")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(oracle["t"], oracle["slip"], label="oracle", color="tab:blue")
    axes[1].plot(sil["t"], sil["slip"], label="SIL", color="tab:orange")
    axes[1].axhspan(0.10, 0.20, color="gray", alpha=0.15, label="target zone")
    axes[1].set_ylabel("slip ratio")
    axes[1].set_title("Slip — both controllers stay around the target zone")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(oracle["t"], oracle["brake_cmd"], label="oracle", color="tab:blue")
    axes[2].plot(sil["t"], sil["brake_cmd"], label="SIL", color="tab:orange")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("brake (bar)")
    axes[2].set_title("Bang-bang command — identical algorithm in two languages")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    # Annotate stopping distances
    d_oracle = sum(0.5 * (oracle["v"][i] + oracle["v"][i+1]) * (oracle["t"][i+1] - oracle["t"][i])
                   for i in range(len(oracle["t"]) - 1))
    d_sil = sum(0.5 * (sil["v"][i] + sil["v"][i+1]) * (sil["t"][i+1] - sil["t"][i])
                for i in range(len(sil["t"]) - 1))
    axes[0].text(0.02, 0.05, f"oracle: {d_oracle:.1f} m\nSIL:    {d_sil:.1f} m",
                 transform=axes[0].transAxes, fontsize=10, family="monospace",
                 verticalalignment="bottom",
                 bbox=dict(facecolor="white", alpha=0.8))

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  saved -> {out}")


# --- plot 2: scenarios summary ----------------------------------------------

SCENARIOS = ["s1_nominal", "s2_ice_patch", "s3_stuck_sensor",
             "s4_comm_loss", "s5_crc_corruption", "s6_noisy_sensor"]
SCEN_LABELS = ["Nominal", "Ice patch", "Stuck sensor",
               "Comm loss 200ms", "CRC corruption", "Noisy sensor"]


def plot_scenarios_summary(out: Path) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    distances = []
    colors = []
    for s in SCENARIOS:
        log = load_csv(SCEN_DIR / f"{s}.csv")
        # Integrate v(t) to recover distance.
        d = sum(0.5 * (log["v"][i] + log["v"][i+1]) * (log["t"][i+1] - log["t"][i])
                for i in range(len(log["t"]) - 1))
        distances.append(d)
        latched = any(s == P.ECU_FAULT_LATCHED for s in log["state"])
        colors.append("tab:red" if latched else "tab:green")

    bars = ax.bar(SCEN_LABELS, distances, color=colors)
    ax.axhline(60.0, color="gray", linestyle="--", alpha=0.5, label="no-ABS baseline (~60 m)")
    ax.axhline(50.0, color="green", linestyle="--", alpha=0.5, label="nominal-ABS target (~50 m)")
    for b, d in zip(bars, distances):
        ax.text(b.get_x() + b.get_width()/2, b.get_height() + 3, f"{d:.1f} m",
                ha="center", fontsize=10)
    ax.set_ylabel("stopping distance (m)")
    ax.set_title("Stopping distance per scenario — red = FAULT_LATCHED reached")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3, axis="y")
    plt.xticks(rotation=10)

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  saved -> {out}")


# --- plot 3: DTC timeline ---------------------------------------------------

def plot_dtc_timeline(out: Path) -> None:
    fig, axes = plt.subplots(len(SCENARIOS), 1, figsize=(11, 1.8 * len(SCENARIOS)),
                             sharex=False)
    for ax, s, label in zip(axes, SCENARIOS, SCEN_LABELS):
        log = load_csv(SCEN_DIR / f"{s}.csv")
        for bit_value, name in DTC_NAMES:
            bits = [1 if (d & bit_value) else 0 for d in log["dtc"]]
            if any(bits):
                # plot tiny coloured rectangles where the DTC is raised
                ax.fill_between(log["t"], 0, bits, step="post", alpha=0.6, label=name)
        ax.set_ylim(0, 1.2)
        ax.set_yticks([])
        ax.set_ylabel(label, fontsize=9, rotation=0, ha="right", va="center")
        ax.grid(True, alpha=0.2, axis="x")
        if ax.has_data():
            ax.legend(loc="upper right", fontsize=7)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("DTC bitfield over time — per scenario", fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  saved -> {out}")


# --- main ------------------------------------------------------------------

def main() -> int:
    RESULTS.mkdir(parents=True, exist_ok=True)
    print("Generating comparison plots...")
    plot_oracle_vs_sil(RESULTS / "oracle_vs_c_sil.png")
    plot_scenarios_summary(RESULTS / "scenarios_summary.png")
    plot_dtc_timeline(RESULTS / "dtc_timeline.png")
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
