"""Reference ABS controller — bang-bang with hysteresis.

This is NOT the controller that will run on the embedded target (that one will
live in C). It is the same algorithm, implemented in Python, that we use as an
"oracle" to validate the plant model in week 1.

Algorithm (same as the C controller of week 2):

    if slip > UPPER:  release brake     (apply 0 bar)
    elif slip < LOWER: re-apply driver request
    else:              hold previous command (hysteresis)

UPPER/LOWER straddle the peak of the mu(s) curve (~0.15 on dry asphalt).
"""

from __future__ import annotations


SLIP_UPPER_THRESHOLD = 0.20
SLIP_LOWER_THRESHOLD = 0.10
LOW_SPEED_BYPASS = 5.0 / 3.6   # m/s — below this the ABS hands over to driver


class OracleABS:
    """Bang-bang with hysteresis, written in Python for plant validation."""

    def __init__(self,
                 upper: float = SLIP_UPPER_THRESHOLD,
                 lower: float = SLIP_LOWER_THRESHOLD,
                 low_speed_bypass: float = LOW_SPEED_BYPASS) -> None:
        self.upper = upper
        self.lower = lower
        self.low_speed_bypass = low_speed_bypass
        self._previous_command = 0.0

    def reset(self) -> None:
        self._previous_command = 0.0

    def step(self, slip: float, v_vehicle: float, driver_request: float) -> float:
        """Return brake pressure command (bar)."""
        if v_vehicle < self.low_speed_bypass:
            # At crawl speed, slip ratio is meaningless — pass the pedal through.
            self._previous_command = driver_request
            return driver_request

        if slip > self.upper:
            cmd = 0.0
        elif slip < self.lower:
            cmd = driver_request
        else:
            cmd = self._previous_command

        self._previous_command = cmd
        return cmd
