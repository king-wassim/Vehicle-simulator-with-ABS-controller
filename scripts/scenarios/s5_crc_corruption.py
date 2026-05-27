"""Scenario 5 — Bit flip on the wire (FMEA F04).

1 % of frames get a corrupted CRC during the run. Expected:
  - frames are silently rejected at the receiver (no decoded payload)
  - system continues — most frames are fine, controller still functions
  - voiture freine normalement, distance reste dans l'envelope
"""

from __future__ import annotations

import sys

from plant import protocol as P
from plant.fault_injector import CrcCorruptionFault, FaultInjector

from scripts.scenarios.harness import (
    ScenarioConfig, RESULTS_DIR, assert_all, print_summary, run_scenario,
)


def main() -> int:
    cfg = ScenarioConfig(name="s5_crc_corruption", surface="dry_asphalt",
                         duration_s=6.0, driver_request_bar=100.0)
    inj = FaultInjector().add(
        CrcCorruptionFault(t_start_s=0.0, t_end_s=1e9,
                           every_n=100, flip_byte_offset=7))
    r = run_scenario(cfg, inj)
    r.to_csv(RESULTS_DIR / "scenarios" / "s5_crc_corruption.csv")
    print_summary(r)

    # No persistent DTC expected for 1 % rate (below threshold of 5/100).
    # COMM_TIMEOUT may flash briefly if the *last* frame before a recv was
    # the corrupted one and the next arrived after the 50 ms watchdog.
    return assert_all([
        (P.ECU_ACTIVE in r.states_visited(),
         "controller still engaged ACTIVE despite corruption"),
        (35.0 <= r.stopping_distance() <= 60.0,
         f"stopping distance {r.stopping_distance():.1f} m close to nominal"),
        (P.ECU_FAULT_LATCHED not in r.states_visited(),
         "ECU did not latch (single-bit flips are isolated events)"),
    ])


if __name__ == "__main__":
    sys.exit(main())
