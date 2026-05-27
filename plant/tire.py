"""Pacejka simplified tire model.

Implements the longitudinal Magic Formula:

    mu(s) = D * sin(C * atan(B * s))

Reference: PHYSICS.md section 3.

The model is a pure function of slip ratio; road conditions are captured by
swapping the (B, C, D) coefficients via the SURFACES presets.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TireParams:
    """Pacejka simplified coefficients (longitudinal)."""

    B: float  # Stiffness — initial slope of mu(s)
    C: float  # Shape factor — sharpness of the peak
    D: float  # Peak friction coefficient mu_max


# Pacejka shape factor C for longitudinal slip is ~1.65 (Bakker/Pacejka 1989);
# values closer to 2 are used for *lateral* slip. With C=1.65 and B=10 the
# peak sits at s≈0.14 and mu(s=1) ≈ 0.65, which matches the standard
# "locked wheel friction is ~65% of peak" used to validate the stopping
# distance envelopes in PHYSICS.md §5.
SURFACES: dict[str, TireParams] = {
    "dry_asphalt":   TireParams(B=10.0, C=1.65, D=1.00),
    "wet_asphalt":   TireParams(B=12.0, C=1.65, D=0.70),
    "packed_snow":   TireParams(B=5.0,  C=1.65, D=0.30),
    "ice":           TireParams(B=4.0,  C=1.65, D=0.10),
}


class PacejkaTire:
    """Longitudinal Pacejka tire — slip in, friction coefficient out."""

    def __init__(self, surface: str = "dry_asphalt") -> None:
        if surface not in SURFACES:
            raise ValueError(
                f"unknown surface '{surface}'; choices: {list(SURFACES)}"
            )
        self._surface = surface
        self._params = SURFACES[surface]

    @property
    def surface(self) -> str:
        return self._surface

    @property
    def params(self) -> TireParams:
        return self._params

    def set_surface(self, surface: str) -> None:
        """Hot-swap road conditions (used by the fault injector — ice patch)."""
        if surface not in SURFACES:
            raise ValueError(
                f"unknown surface '{surface}'; choices: {list(SURFACES)}"
            )
        self._surface = surface
        self._params = SURFACES[surface]

    def mu(self, slip: float) -> float:
        """Friction coefficient mu for a given slip ratio.

        Signed: returns negative mu for negative slip (traction case). For pure
        braking the simulator clamps slip to [0, 1], so this branch is only
        exercised by unit tests.
        """
        p = self._params
        return p.D * math.sin(p.C * math.atan(p.B * slip))

    def peak_slip(self, resolution: int = 1000) -> float:
        """Slip ratio at which mu(s) is maximal — handy for sanity checks."""
        best_s, best_mu = 0.0, 0.0
        for i in range(1, resolution + 1):
            s = i / resolution
            m = self.mu(s)
            if m > best_mu:
                best_mu, best_s = m, s
        return best_s


def compute_slip(v_vehicle: float, omega_wheel: float, wheel_radius: float,
                 v_min: float = 1.0) -> float:
    """Longitudinal slip ratio s = (v - omega*R) / v.

    Returns 0 when |v| < v_min to avoid the singularity at standstill (see
    PHYSICS.md §2.3 and FMEA mode F06). Output is clamped to [0, 1] because the
    plant model is pure braking — wheel cannot spin faster than the car body.
    """
    if abs(v_vehicle) < v_min:
        return 0.0
    s = (v_vehicle - omega_wheel * wheel_radius) / v_vehicle
    if s < 0.0:
        return 0.0
    if s > 1.0:
        return 1.0
    return s
