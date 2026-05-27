"""Scenario 1 — Nominal panic braking on dry asphalt (baseline).

No faults. Expected:
  - stopping distance in [35, 55] m
  - ECU reaches ACTIVE
  - 0 DTC raised
"""

from __future__ import annotations

import sys

from plant import protocol as P
from plant.fault_injector import FaultInjector

from scripts.scenarios.harness import (
    ScenarioConfig, RESULTS_DIR, assert_all, print_summary, run_scenario,
)


def main() -> int:
    cfg = ScenarioConfig(name="s1_nominal", surface="dry_asphalt",
                         duration_s=6.0, driver_request_bar=100.0)
    inj = FaultInjector()
    r = run_scenario(cfg, inj)
    r.to_csv(RESULTS_DIR / "scenarios" / "s1_nominal.csv")
    print_summary(r)
    return assert_all([
        (35.0 <= r.stopping_distance() <= 55.0,
         f"stopping distance {r.stopping_distance():.1f} m in [35, 55]"),
        (P.ECU_ACTIVE in r.states_visited(),
         "ECU reached ACTIVE"),
        (r.dtc_union() == 0,
         f"no DTC raised (got 0x{r.dtc_union():04X})"),
    ])


if __name__ == "__main__":
    sys.exit(main())
