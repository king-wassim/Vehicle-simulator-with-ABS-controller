"""Quarter-car longitudinal dynamics.

State variables (PHYSICS.md §4):
    v       : vehicle longitudinal velocity                       [m/s]
    omega   : wheel angular velocity                              [rad/s]
    x       : travelled distance                                  [m]

Inputs:
    brake_pressure : hydraulic line pressure commanded by ECU     [bar]

The integrator is explicit Euler at dt = 1 ms (much smaller than the ECU
control period of 10 ms). At dt <= 1 ms the model is numerically stable for
mu in [0, 1] and brake torques up to wheel-locking; if dt is increased above
~5 ms the wheel speed can overshoot below zero in a single step on icy
surfaces. We clamp omega >= 0 as a guard rather than introduce RK4 — the model
is only used in SIL, not on the real ECU.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .tire import PacejkaTire, compute_slip


G = 9.81  # m/s^2

# Below this speed, slip ratio is ill-defined (compute_slip() returns 0 below
# its own v_min = 1.0 m/s, which kills kinetic friction). To avoid coasting
# forever on a zero-mu plateau, snap the state to standstill once we drop
# under this threshold *and* the brake is being commanded — physically this
# models the transition from kinetic to static friction at near-zero speed.
# Must be >= compute_slip()'s v_min, otherwise the model gets stuck.
V_STATIC_FREEZE = 1.0  # m/s


@dataclass
class VehicleParams:
    """Quarter-car physical parameters."""

    mass_quarter: float = 400.0       # kg  (1500 kg full car / ~quarter mass on a braked axle)
    wheel_radius: float = 0.30        # m
    wheel_inertia: float = 1.2        # kg.m^2
    brake_gain: float = 25.0          # N.m / bar — torque produced per bar of line pressure
    max_brake_pressure: float = 150.0 # bar


@dataclass
class VehicleState:
    v: float = 27.78        # m/s    (100 km/h)
    omega: float = 0.0      # rad/s  (set in __post_init__ via Vehicle)
    x: float = 0.0          # m
    t: float = 0.0          # s


@dataclass
class StepOutputs:
    """Snapshot of derived signals at the end of a step — useful for logging."""

    slip: float
    mu: float
    brake_torque: float
    brake_pressure: float


class Vehicle:
    """Quarter-car plant with Pacejka tire and hydraulic brake."""

    def __init__(self,
                 params: VehicleParams | None = None,
                 tire: PacejkaTire | None = None,
                 initial_speed: float = 27.78) -> None:
        self.params = params or VehicleParams()
        self.tire = tire or PacejkaTire("dry_asphalt")
        self.state = VehicleState(v=initial_speed)
        # Wheel starts free-rolling: omega * R = v
        self.state.omega = initial_speed / self.params.wheel_radius

    def reset(self, initial_speed: float = 27.78) -> None:
        self.state = VehicleState(v=initial_speed)
        self.state.omega = initial_speed / self.params.wheel_radius

    def step(self, brake_pressure: float, dt: float = 0.001) -> StepOutputs:
        """Advance the state by `dt` seconds under the given brake pressure.

        Returns the derived signals computed during the step so callers can log
        them without recomputing.
        """
        p = self.params
        s = self.state

        # Saturate command into the physical actuator range.
        if brake_pressure < 0.0:
            brake_pressure = 0.0
        elif brake_pressure > p.max_brake_pressure:
            brake_pressure = p.max_brake_pressure

        slip = compute_slip(s.v, s.omega, p.wheel_radius)
        mu = self.tire.mu(slip)

        # Longitudinal friction force at the contact patch (quarter car).
        F = mu * p.mass_quarter * G
        brake_torque = p.brake_gain * brake_pressure

        # Newton on the body and on the wheel (PHYSICS.md §4.3 / §4.4).
        dv = -(F / p.mass_quarter) * dt
        domega = ((F * p.wheel_radius) - brake_torque) / p.wheel_inertia * dt

        s.v += dv
        s.omega += domega
        s.x += s.v * dt + 0.5 * dv * dt  # trapezoidal distance update
        s.t += dt

        # Physical clamps — the vehicle does not roll backwards and the wheel
        # does not spin in reverse under pure braking.
        if s.v < 0.0:
            s.v = 0.0
        if s.omega < 0.0:
            s.omega = 0.0

        # Low-speed transition to static friction (see V_STATIC_FREEZE above).
        if s.v < V_STATIC_FREEZE and brake_pressure > 1.0:
            s.v = 0.0
            s.omega = 0.0

        return StepOutputs(slip=slip, mu=mu,
                           brake_torque=brake_torque,
                           brake_pressure=brake_pressure)

    @property
    def stopped(self) -> bool:
        return self.state.v < 1e-3
