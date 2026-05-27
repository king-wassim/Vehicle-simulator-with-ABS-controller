"""Scenario 4 — Transient communication loss (FMEA F03).

200 ms of dropped sensor frames after 1 s. Expected:
  - DTC_COMM_TIMEOUT levée pendant la coupure (watchdog 50 ms)
  - ECU bascule en FAULT_DEGRADED, latch FAULT_LATCHED après 10 cycles
    (20 cycles de coupure dépassent le seuil — comportement correct ISO 26262)
  - voiture s'arrête (fail-operational, la pédale conducteur passe quand même)
"""

from __future__ import annotations

import sys

from plant import protocol as P
from plant.fault_injector import CommLossFault, FaultInjector

from scripts.scenarios.harness import (
    ScenarioConfig, RESULTS_DIR, assert_all, print_summary, run_scenario,
)


def main() -> int:
    cfg = ScenarioConfig(name="s4_comm_loss", surface="dry_asphalt",
                         duration_s=8.0, driver_request_bar=100.0)
    inj = FaultInjector().add(
        CommLossFault(t_start_s=1.0, t_end_s=1.2))   # 200 ms gap
    r = run_scenario(cfg, inj)
    r.to_csv(RESULTS_DIR / "scenarios" / "s4_comm_loss.csv")
    print_summary(r)

    t_first_timeout = r.first_dtc_time(P.DTC_COMM_TIMEOUT)

    return assert_all([
        (t_first_timeout is not None,
         f"DTC_COMM_TIMEOUT raised (first at t={t_first_timeout})"),
        (P.ECU_FAULT_DEGRADED in r.states_visited(),
         "ECU entered FAULT_DEGRADED during the comm gap"),
        (P.ECU_FAULT_LATCHED in r.states_visited(),
         "ECU latched after 10 consecutive faults (correct per FMEA F08)"),
        (r.stopping_distance() < 200.0,
         f"vehicle still stops fail-operational ({r.stopping_distance():.1f} m)"),
    ])


if __name__ == "__main__":
    sys.exit(main())
