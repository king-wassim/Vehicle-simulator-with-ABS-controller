"""Wire protocol — Python mirror of controller/src/drivers/protocol.h.

This module is the *single source of truth* on the Python side for:
  - the binary layout of sensor_payload / actuator_payload
  - the CRC-16/CCITT-FALSE used end-to-end
  - the framing envelope (magic + type + length + payload + crc)

Any change here MUST be mirrored in protocol.h and vice-versa. The unit tests
in tests/test_protocol.py pin the layout sizes so divergence trips CI.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Tuple


# --- Constants (mirrored from protocol.h) ----------------------------------

MAGIC = 0xAA55
TYPE_SENSOR = 0x01
TYPE_ACTUATOR = 0x02
MAX_PAYLOAD = 64

HEADER_SIZE = 5     # magic(2) + type(1) + length(2)
CRC_SIZE = 2
OVERHEAD = HEADER_SIZE + CRC_SIZE

# fault_flags bitfield
FF_INJECTED_STUCK = 1 << 0
FF_INJECTED_NOISE = 1 << 1
FF_INJECTED_RANGE = 1 << 2
FF_INJECTED_ICE   = 1 << 3

# ECU status enum
ECU_INIT = 0
ECU_STANDBY = 1
ECU_MONITOR = 2
ECU_ACTIVE = 3
ECU_FAULT_DEGRADED = 4
ECU_FAULT_LATCHED = 5

# DTC bitfield (FMEA-aligned)
DTC_NONE = 0x0000
DTC_SENSOR_STUCK = 1 << 0
DTC_SENSOR_NOISE = 1 << 1
DTC_COMM_TIMEOUT = 1 << 2
DTC_COMM_CRC     = 1 << 3
DTC_SENSOR_RANGE = 1 << 4
DTC_PLAUSIBILITY = 1 << 5


# --- Packed payload layouts ------------------------------------------------

# Little-endian, no padding. struct format strings must match protocol.h.
SENSOR_FMT = "<IfffH"     # u32 timestamp, f32 v, f32 omega, f32 brake_p, u16 flags
ACTUATOR_FMT = "<IfHH"    # u32 timestamp, f32 brake_cmd, u16 status, u16 dtc

SENSOR_SIZE = struct.calcsize(SENSOR_FMT)
ACTUATOR_SIZE = struct.calcsize(ACTUATOR_FMT)

assert SENSOR_SIZE == 18, f"sensor payload drifted: {SENSOR_SIZE}"
assert ACTUATOR_SIZE == 12, f"actuator payload drifted: {ACTUATOR_SIZE}"


@dataclass
class SensorFrame:
    timestamp_ms: int
    v_vehicle: float
    omega_wheel: float
    brake_pressure: float
    fault_flags: int = 0

    def pack_payload(self) -> bytes:
        return struct.pack(SENSOR_FMT,
                           self.timestamp_ms & 0xFFFFFFFF,
                           self.v_vehicle,
                           self.omega_wheel,
                           self.brake_pressure,
                           self.fault_flags & 0xFFFF)

    @classmethod
    def unpack_payload(cls, data: bytes) -> "SensorFrame":
        if len(data) != SENSOR_SIZE:
            raise ValueError(f"sensor payload wrong size: got {len(data)}, want {SENSOR_SIZE}")
        ts, v, w, bp, ff = struct.unpack(SENSOR_FMT, data)
        return cls(ts, v, w, bp, ff)


@dataclass
class ActuatorFrame:
    timestamp_ms: int
    brake_command: float
    ecu_status: int
    dtc_code: int

    def pack_payload(self) -> bytes:
        return struct.pack(ACTUATOR_FMT,
                           self.timestamp_ms & 0xFFFFFFFF,
                           self.brake_command,
                           self.ecu_status & 0xFFFF,
                           self.dtc_code & 0xFFFF)

    @classmethod
    def unpack_payload(cls, data: bytes) -> "ActuatorFrame":
        if len(data) != ACTUATOR_SIZE:
            raise ValueError(f"actuator payload wrong size: got {len(data)}, want {ACTUATOR_SIZE}")
        ts, bc, st, dtc = struct.unpack(ACTUATOR_FMT, data)
        return cls(ts, bc, st, dtc)


# --- CRC-16/CCITT-FALSE ----------------------------------------------------

def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE — polynomial 0x1021, init 0xFFFF, no reflection.

    Same bit-shift implementation as controller/src/drivers/crc.c so both
    sides match byte-for-byte. KAT: crc16_ccitt(b"123456789") == 0x29B1.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# --- Framing ---------------------------------------------------------------

def encode_frame(frame_type: int, payload: bytes) -> bytes:
    """Wrap a payload in the protocol envelope.

    Wire layout: magic(2) | type(1) | length(2) | payload(N) | crc(2).
    CRC is computed over type + length + payload (magic excluded by design).
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large ({len(payload)} > {MAX_PAYLOAD})")
    if frame_type not in (TYPE_SENSOR, TYPE_ACTUATOR):
        raise ValueError(f"unknown frame type 0x{frame_type:02X}")

    header = struct.pack("<HBH", MAGIC, frame_type, len(payload))
    crc = crc16_ccitt(header[2:] + payload)         # type+length+payload
    return header + payload + struct.pack("<H", crc)


@dataclass
class DecodedFrame:
    frame_type: int
    payload: bytes


class FrameDecoder:
    """Stream decoder with magic-resync — see PROTOCOL.md §3.2.

    Feed `feed(bytes)` chunks of any size; pull whole frames out of
    `drain()`. Bad frames (size out of range, CRC mismatch) are silently
    skipped and counted. Caller queries `errors_crc`, `errors_size`, etc.,
    for telemetry.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.errors_crc = 0
        self.errors_size = 0
        self.errors_type = 0
        self.bytes_dropped = 0

    def feed(self, data: bytes) -> None:
        self._buf.extend(data)

    def drain(self) -> list[DecodedFrame]:
        out: list[DecodedFrame] = []
        while True:
            frame = self._try_one()
            if frame is None:
                break
            out.append(frame)
        return out

    def _try_one(self) -> DecodedFrame | None:
        b = self._buf
        # Find magic in the buffer; if none, keep at most 1 byte (next chunk
        # may complete the magic pair).
        i = 0
        while i + 1 < len(b):
            if b[i] == (MAGIC & 0xFF) and b[i + 1] == (MAGIC >> 8) & 0xFF:
                break
            i += 1
        else:
            # No magic found — discard everything but a possible dangling byte.
            if len(b) > 0:
                self.bytes_dropped += len(b) - (1 if len(b) >= 1 else 0)
                del b[:len(b) - 1]
            return None

        if i > 0:
            self.bytes_dropped += i
            del b[:i]

        # Need at least header + crc bytes to decode anything.
        if len(b) < HEADER_SIZE + CRC_SIZE:
            return None

        frame_type = b[2]
        length = b[3] | (b[4] << 8)

        if length > MAX_PAYLOAD:
            self.errors_size += 1
            del b[:2]              # drop the bad magic, scan past it
            return self._try_one()
        if frame_type not in (TYPE_SENSOR, TYPE_ACTUATOR):
            self.errors_type += 1
            del b[:2]
            return self._try_one()

        total = HEADER_SIZE + length + CRC_SIZE
        if len(b) < total:
            return None            # incomplete — wait for more bytes

        payload = bytes(b[HEADER_SIZE:HEADER_SIZE + length])
        crc_recv = b[HEADER_SIZE + length] | (b[HEADER_SIZE + length + 1] << 8)
        crc_calc = crc16_ccitt(bytes(b[2:HEADER_SIZE + length]))

        if crc_recv != crc_calc:
            self.errors_crc += 1
            del b[:2]              # bad — slide past magic and keep scanning
            return self._try_one()

        del b[:total]
        return DecodedFrame(frame_type=frame_type, payload=payload)


def decode_one(buf: bytes) -> Tuple[DecodedFrame | None, int]:
    """Convenience for tests: decode at most one frame, return (frame, bytes_consumed)."""
    dec = FrameDecoder()
    dec.feed(buf)
    frames = dec.drain()
    if not frames:
        return None, 0
    # FrameDecoder consumes from its internal buffer; we replicate that here.
    # For test simplicity we just compute via the leftover length.
    consumed = len(buf) - len(dec._buf) - dec.bytes_dropped
    return frames[0], consumed
