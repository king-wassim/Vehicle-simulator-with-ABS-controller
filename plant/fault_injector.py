"""Programmable fault injection — companion to docs/FMEA.md.

The injector sits between the plant's "real" sensor values and the wire
frame the plant ships to the ECU. Each fault type from the FMEA has a
corresponding hook:

    F01 — stuck wheel sensor:    StuckSensorFault
    F02 — gaussian sensor noise: NoisySensorFault
    F05 — out-of-range value:    RangeViolationFault
    F03 — dropped frame:         CommLossFault     (returns None to skip the send)
    F04 — CRC corruption:        CrcCorruptionFault (toggles bytes in the wire)
    F10 — ice patch mid-run:     IcePatchFault     (mutates the tire surface)

Each fault has an `active` window expressed in (t_start_s, t_end_s) so a
scenario can stage them: "go nominal for 1 s, then ice from 1-3 s, then
stuck from 2-2.5 s overlapping with ice", etc.

The injector is used by the scenario scripts under scripts/scenarios/. It
does NOT depend on the controller — it lives entirely on the plant side.
"""

from __future__ import annotations

import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from . import protocol as P
from .tire import PacejkaTire


# --- base class -------------------------------------------------------------

@dataclass
class Fault(ABC):
    """A fault active only between `t_start_s` and `t_end_s` (inclusive)."""

    t_start_s: float = 0.0
    t_end_s: float = math.inf
    name: str = "fault"

    def is_active(self, t_s: float) -> bool:
        return self.t_start_s <= t_s <= self.t_end_s

    @abstractmethod
    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> Optional[P.SensorFrame]:
        """Mutate (or replace) the sensor frame about to be sent. Return None
        to drop the frame entirely (used by CommLoss)."""
        return frame


# --- F01 stuck wheel sensor ------------------------------------------------

@dataclass
class StuckSensorFault(Fault):
    name: str = "stuck_sensor"
    _frozen_omega: Optional[float] = None

    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> P.SensorFrame:
        if not self.is_active(t_s):
            self._frozen_omega = None
            return frame
        if self._frozen_omega is None:
            self._frozen_omega = frame.omega_wheel
        return P.SensorFrame(
            timestamp_ms=frame.timestamp_ms,
            v_vehicle=frame.v_vehicle,
            omega_wheel=self._frozen_omega,
            brake_pressure=frame.brake_pressure,
            fault_flags=frame.fault_flags | P.FF_INJECTED_STUCK,
        )


# --- F02 noisy wheel sensor ------------------------------------------------

@dataclass
class NoisySensorFault(Fault):
    name: str = "noisy_sensor"
    sigma_rad_per_s: float = 5.0
    rng: random.Random = field(default_factory=lambda: random.Random(42))

    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> P.SensorFrame:
        if not self.is_active(t_s):
            return frame
        return P.SensorFrame(
            timestamp_ms=frame.timestamp_ms,
            v_vehicle=frame.v_vehicle,
            omega_wheel=frame.omega_wheel
                        + self.rng.gauss(0.0, self.sigma_rad_per_s),
            brake_pressure=frame.brake_pressure,
            fault_flags=frame.fault_flags | P.FF_INJECTED_NOISE,
        )


# --- F05 range violation ---------------------------------------------------

@dataclass
class RangeViolationFault(Fault):
    """Replace omega with an aberrant value (e.g. -999 or +9999) to test
    the range check downstream."""

    name: str = "range_violation"
    bad_omega: float = -999.0

    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> P.SensorFrame:
        if not self.is_active(t_s):
            return frame
        return P.SensorFrame(
            timestamp_ms=frame.timestamp_ms,
            v_vehicle=frame.v_vehicle,
            omega_wheel=self.bad_omega,
            brake_pressure=frame.brake_pressure,
            fault_flags=frame.fault_flags | P.FF_INJECTED_RANGE,
        )


# --- F03 dropped frames ----------------------------------------------------

@dataclass
class CommLossFault(Fault):
    """Drop the sensor frame entirely (caller skips the send)."""

    name: str = "comm_loss"

    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> Optional[P.SensorFrame]:
        if self.is_active(t_s):
            return None
        return frame


# --- F04 CRC corruption ----------------------------------------------------

@dataclass
class CrcCorruptionFault(Fault):
    """Periodic bit flip in the wire buffer — `every_n` frames in the active
    window. The flip is applied AFTER framing/CRC so the receiver sees a
    proper CRC mismatch."""

    name: str = "crc_corruption"
    every_n: int = 100        # corrupt 1 frame in every N
    flip_byte_offset: int = 7  # somewhere in the payload, not the magic
    _counter: int = 0

    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> P.SensorFrame:
        # Mutation happens at wire-level — handled by `mutate_wire` below.
        return frame

    def mutate_wire(self, wire: bytes, t_s: float) -> bytes:
        if not self.is_active(t_s):
            return wire
        self._counter += 1
        if self._counter % self.every_n != 0:
            return wire
        if self.flip_byte_offset >= len(wire) - 2:
            return wire
        b = bytearray(wire)
        b[self.flip_byte_offset] ^= 0xFF
        return bytes(b)


# --- F10 ice patch (env, not signal) ---------------------------------------

@dataclass
class IcePatchFault(Fault):
    """Mutates the *tire* (not the signal) at run-time — the world really
    becomes slippery. The scenario applies this by holding a reference to
    the Vehicle's PacejkaTire."""

    name: str = "ice_patch"
    target_surface: str = "ice"
    _restore: Optional[str] = None

    def apply_in_plant(self, tire: PacejkaTire, t_s: float) -> None:
        if self.is_active(t_s):
            if self._restore is None:
                self._restore = tire.surface
            if tire.surface != self.target_surface:
                tire.set_surface(self.target_surface)
        else:
            if self._restore is not None and tire.surface != self._restore:
                tire.set_surface(self._restore)
                self._restore = None

    def apply_to_sensor(self, frame: P.SensorFrame, t_s: float) -> P.SensorFrame:
        # Surface change is applied via apply_in_plant; signal is not modified.
        return frame


# --- composer --------------------------------------------------------------

@dataclass
class FaultInjector:
    """Chains multiple faults — applied in order. Stops at the first one
    that returns None (CommLoss aborts the frame)."""

    faults: list[Fault] = field(default_factory=list)

    def add(self, fault: Fault) -> "FaultInjector":
        self.faults.append(fault)
        return self

    def filter_sensor(self, frame: P.SensorFrame, t_s: float) -> Optional[P.SensorFrame]:
        out: Optional[P.SensorFrame] = frame
        for f in self.faults:
            if out is None:
                return None
            out = f.apply_to_sensor(out, t_s)
        return out

    def filter_wire(self, wire: bytes, t_s: float) -> bytes:
        out = wire
        for f in self.faults:
            if isinstance(f, CrcCorruptionFault):
                out = f.mutate_wire(out, t_s)
        return out

    def apply_environment(self, tire: PacejkaTire, t_s: float) -> None:
        for f in self.faults:
            if isinstance(f, IcePatchFault):
                f.apply_in_plant(tire, t_s)

    def active_names(self, t_s: float) -> list[str]:
        return [f.name for f in self.faults if f.is_active(t_s)]
