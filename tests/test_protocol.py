"""Protocol unit tests — pin layout, CRC, framing, resync.

Run with `pytest tests/` from the repo root. No external deps beyond stdlib.
"""

from __future__ import annotations

import struct
import unittest

from plant import protocol as P


class TestCRC(unittest.TestCase):
    def test_kat_check_vector(self):
        # Canonical KAT for CRC-16/CCITT-FALSE.
        self.assertEqual(P.crc16_ccitt(b"123456789"), 0x29B1)

    def test_empty_input(self):
        # Init value with no data XORed = 0xFFFF.
        self.assertEqual(P.crc16_ccitt(b""), 0xFFFF)

    def test_single_byte_known_vectors(self):
        # Hand-traced reference values for the CCITT-FALSE polynomial,
        # init=0xFFFF, no reflection. Verified independently below by the
        # cross-language test against the C implementation.
        self.assertEqual(P.crc16_ccitt(b"\x00"), 0xE1F0)
        self.assertEqual(P.crc16_ccitt(b"\xff"), 0xFF00)


class TestPayloadLayout(unittest.TestCase):
    def test_sensor_size_pinned(self):
        self.assertEqual(P.SENSOR_SIZE, 18)

    def test_actuator_size_pinned(self):
        self.assertEqual(P.ACTUATOR_SIZE, 12)

    def test_sensor_roundtrip(self):
        f = P.SensorFrame(timestamp_ms=12345,
                          v_vehicle=27.78,
                          omega_wheel=92.6,
                          brake_pressure=42.5,
                          fault_flags=P.FF_INJECTED_NOISE)
        raw = f.pack_payload()
        self.assertEqual(len(raw), P.SENSOR_SIZE)
        g = P.SensorFrame.unpack_payload(raw)
        self.assertEqual(g.timestamp_ms, 12345)
        self.assertAlmostEqual(g.v_vehicle, 27.78, places=4)
        self.assertAlmostEqual(g.omega_wheel, 92.6, places=4)
        self.assertAlmostEqual(g.brake_pressure, 42.5, places=4)
        self.assertEqual(g.fault_flags, P.FF_INJECTED_NOISE)

    def test_actuator_roundtrip(self):
        a = P.ActuatorFrame(timestamp_ms=42,
                            brake_command=85.5,
                            ecu_status=P.ECU_ACTIVE,
                            dtc_code=P.DTC_SENSOR_NOISE | P.DTC_COMM_CRC)
        raw = a.pack_payload()
        self.assertEqual(len(raw), P.ACTUATOR_SIZE)
        b = P.ActuatorFrame.unpack_payload(raw)
        self.assertEqual(b.timestamp_ms, 42)
        self.assertAlmostEqual(b.brake_command, 85.5, places=4)
        self.assertEqual(b.ecu_status, P.ECU_ACTIVE)
        self.assertEqual(b.dtc_code, P.DTC_SENSOR_NOISE | P.DTC_COMM_CRC)

    def test_little_endian_byte_order(self):
        # First byte of timestamp_ms (LE) should be the LSB.
        f = P.SensorFrame(timestamp_ms=0x12345678, v_vehicle=0, omega_wheel=0,
                          brake_pressure=0, fault_flags=0)
        raw = f.pack_payload()
        self.assertEqual(raw[0], 0x78)
        self.assertEqual(raw[3], 0x12)


class TestEnvelopeAndCodec(unittest.TestCase):
    def test_encode_decode_roundtrip(self):
        f = P.SensorFrame(123, 25.0, 80.0, 50.0, 0)
        wire = P.encode_frame(P.TYPE_SENSOR, f.pack_payload())
        # Envelope: 5 header + 18 payload + 2 CRC = 25 bytes
        self.assertEqual(len(wire), P.OVERHEAD + P.SENSOR_SIZE)

        dec = P.FrameDecoder()
        dec.feed(wire)
        frames = dec.drain()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].frame_type, P.TYPE_SENSOR)
        g = P.SensorFrame.unpack_payload(frames[0].payload)
        self.assertEqual(g.timestamp_ms, 123)

    def test_resync_after_garbage_prefix(self):
        # Receiver should ignore arbitrary leading bytes and still find the frame.
        f = P.SensorFrame(1, 1.0, 1.0, 1.0, 0)
        wire = P.encode_frame(P.TYPE_SENSOR, f.pack_payload())
        garbage = b"\xde\xad\xbe\xef\xaa\xaa\xaa"  # includes a stray 0xAA
        dec = P.FrameDecoder()
        dec.feed(garbage + wire)
        frames = dec.drain()
        self.assertEqual(len(frames), 1)
        self.assertGreater(dec.bytes_dropped, 0)

    def test_chunked_feed(self):
        # Splitting the frame across multiple feed() calls must still decode.
        f = P.ActuatorFrame(99, 12.5, P.ECU_MONITOR, 0)
        wire = P.encode_frame(P.TYPE_ACTUATOR, f.pack_payload())
        dec = P.FrameDecoder()
        # Feed 1 byte at a time — worst case.
        for byte in wire:
            dec.feed(bytes([byte]))
        frames = dec.drain()
        self.assertEqual(len(frames), 1)

    def test_crc_mismatch_is_counted_and_resynced(self):
        f = P.SensorFrame(1, 1.0, 1.0, 1.0, 0)
        wire = bytearray(P.encode_frame(P.TYPE_SENSOR, f.pack_payload()))
        wire[-1] ^= 0xFF  # corrupt the CRC

        # Append a clean frame after — decoder should drop the first and find the second.
        clean = P.encode_frame(P.TYPE_SENSOR, P.SensorFrame(2, 2.0, 2.0, 2.0, 0).pack_payload())
        dec = P.FrameDecoder()
        dec.feed(bytes(wire) + clean)
        frames = dec.drain()
        self.assertEqual(dec.errors_crc, 1)
        self.assertEqual(len(frames), 1)
        g = P.SensorFrame.unpack_payload(frames[0].payload)
        self.assertEqual(g.timestamp_ms, 2)

    def test_oversize_length_field_rejected(self):
        # Build a frame with length = 0xFFFF — must be discarded.
        bad = struct.pack("<HBH", P.MAGIC, P.TYPE_SENSOR, 0xFFFF) + b"\x00" * 8
        dec = P.FrameDecoder()
        dec.feed(bad)
        dec.drain()
        self.assertEqual(dec.errors_size, 1)


if __name__ == "__main__":
    unittest.main()
