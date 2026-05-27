"""Scenario 2 — Ice patch mid-run (FMEA F10).

Vehicle starts on dry asphalt, hits an ice patch at t=1.0 s. Expected:
  - μ_max drops to 0.1, deceleration falls dramatically
  - vehicle still reaches near-stop (we run long enough)
  - ECU oscillates between MONITOR and ACTIVE but never latches
  - 0 DTC (this is *environment*, not a sensor fault)
"""

from __future__ import annotations

import sys

from plant import protocol as P
from plant.fault_injector import FaultInjector, IcePatchFault

from scripts.scenarios.harness import (
    ScenarioConfig, RESULTS_DIR, assert_all, print_summary, run_scenario,
)


def main() -> int:
    cfg = ScenarioConfig(name="s2_ice_patch", surface="dry_asphalt",
                         duration_s=15.0, driver_request_bar=100.0)
    inj = FaultInjector().add(
        IcePatchFault(t_start_s=1.0, t_end_s=1e9, target_surface="ice"))
    r = run_scenario(cfg, inj)
    r.to_csv(RESULTS_DIR / "scenarios" / "s2_ice_patch.csv")
    print_summary(r)
    return assert_all([
        # Vehicle CAN'T stop in 55m on ice — distance must be significantly
        # higher than the dry-asphalt nominal.
        (r.stopping_distance() > 70.0,
         f"stopping distance {r.stopping_distance():.1f} m > 70 (ice effect)"),
        (P.ECU_FAULT_LATCHED not in r.states_visited(),
         "ECU did not latch (environment, not a fault)"),
        (r.dtc_union() == 0,
         f"no DTC raised (got 0x{r.dtc_union():04X})"),
    ])


if __name__ == "__main__":
    sys.exit(main())
