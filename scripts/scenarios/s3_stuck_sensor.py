"""Scenario 3 — Stuck wheel sensor (FMEA F01).

After 0.5 s of normal braking, the wheel speed sensor freezes. Expected:
  - DTC_SENSOR_STUCK levée en < 250 ms après le début (= 25 cycles)
  - ECU passe en FAULT_DEGRADED puis FAULT_LATCHED si la panne persiste
  - le contrôleur passe la pédale brute — la voiture s'arrête quand même
"""

from __future__ import annotations

import sys

from plant import protocol as P
from plant.fault_injector import FaultInjector, StuckSensorFault

from scripts.scenarios.harness import (
    ScenarioConfig, RESULTS_DIR, assert_all, print_summary, run_scenario,
)


def main() -> int:
    cfg = ScenarioConfig(name="s3_stuck_sensor", surface="dry_asphalt",
                         duration_s=8.0, driver_request_bar=100.0)
    inj = FaultInjector().add(
        StuckSensorFault(t_start_s=0.5, t_end_s=1e9))
    r = run_scenario(cfg, inj)
    r.to_csv(RESULTS_DIR / "scenarios" / "s3_stuck_sensor.csv")
    print_summary(r)

    t_first_stuck = r.first_dtc_time(P.DTC_SENSOR_STUCK)
    delay = (t_first_stuck - 0.5) if t_first_stuck is not None else float("inf")

    return assert_all([
        (t_first_stuck is not None,
         f"DTC_SENSOR_STUCK raised (first at t={t_first_stuck})"),
        (delay < 0.25,
         f"detected within 250 ms (got {delay*1000:.0f} ms after injection)"),
        (P.ECU_FAULT_DEGRADED in r.states_visited()
         or P.ECU_FAULT_LATCHED in r.states_visited(),
         "ECU reached a FAULT_* state"),
        (r.stopping_distance() < 200.0,
         f"vehicle still stops ({r.stopping_distance():.1f} m, fail-operational)"),
    ])


if __name__ == "__main__":
    sys.exit(main())
