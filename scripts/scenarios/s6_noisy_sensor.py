"""Scenario 6 — Noisy wheel sensor (FMEA F02).

Gaussian noise σ = 5 rad/s injected on omega throughout. Expected:
  - bang-bang chatter is more aggressive, distance d'arrêt légèrement allongée
  - DTC_SENSOR_NOISE *would* be raised if the noise detector were enabled
    in the orchestrator — see diagnostic.c F02 block (currently gated, see
    docs/JOURNAL.md W3 J16-17 for the tuning discussion)
  - voiture s'arrête, pas de latch
"""

from __future__ import annotations

import sys

from plant import protocol as P
from plant.fault_injector import FaultInjector, NoisySensorFault

from scripts.scenarios.harness import (
    ScenarioConfig, RESULTS_DIR, assert_all, print_summary, run_scenario,
)


def main() -> int:
    cfg = ScenarioConfig(name="s6_noisy_sensor", surface="dry_asphalt",
                         duration_s=8.0, driver_request_bar=100.0)
    inj = FaultInjector().add(
        NoisySensorFault(t_start_s=0.0, t_end_s=1e9, sigma_rad_per_s=5.0))
    r = run_scenario(cfg, inj)
    r.to_csv(RESULTS_DIR / "scenarios" / "s6_noisy_sensor.csv")
    print_summary(r)

    # Noise σ=5 means occasional ±15 rad/s spikes — those briefly fall
    # outside the [-1, 500] omega range when omega is near 0 during a
    # locked-wheel phase. That's the correct behaviour of the range check,
    # so DTC_SENSOR_RANGE is allowed here. What matters is the global
    # outcome: the vehicle still stops, the ECU doesn't latch.
    return assert_all([
        (r.stopping_distance() < 80.0,
         f"vehicle still stops in <80 m ({r.stopping_distance():.1f} m)"),
        (P.ECU_FAULT_LATCHED not in r.states_visited(),
         "ECU did not latch on noise alone"),
        (P.ECU_ACTIVE in r.states_visited(),
         "controller still engaged ACTIVE despite noise"),
    ])


if __name__ == "__main__":
    sys.exit(main())
