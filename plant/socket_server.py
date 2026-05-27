"""TCP server side of the SIL link.

The plant acts as the server; the ECU connects in. One client at a time —
the SIL bench is point-to-point. A clean handshake / shutdown sequence is
not yet defined (week 1 stops at "1000 frames round-trip"); we treat any
disconnect as terminal and wait for a reconnect on the next call.
"""

from __future__ import annotations

import socket
import time
from dataclasses import dataclass, field

from . import protocol as P


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9000


@dataclass
class LinkStats:
    sensor_frames_sent: int = 0
    actuator_frames_received: int = 0
    errors_crc: int = 0
    errors_size: int = 0
    errors_type: int = 0
    bytes_dropped: int = 0


class PlantLink:
    """One-client TCP server with the binary protocol."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.host = host
        self.port = port
        self._listener: socket.socket | None = None
        self._conn: socket.socket | None = None
        self._decoder = P.FrameDecoder()
        self.stats = LinkStats()

    # --- lifecycle ---------------------------------------------------------

    def listen(self) -> None:
        """Open the listening socket. Idempotent."""
        if self._listener is not None:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(1)
        self._listener = s

    def accept(self, timeout: float | None = None) -> None:
        """Block until the ECU connects. Raises socket.timeout on timeout."""
        assert self._listener is not None, "call listen() first"
        self._listener.settimeout(timeout)
        conn, _ = self._listener.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(None)
        self._conn = conn
        self._decoder = P.FrameDecoder()

    def close(self) -> None:
        if self._conn is not None:
            try: self._conn.close()
            except OSError: pass
            self._conn = None
        if self._listener is not None:
            try: self._listener.close()
            except OSError: pass
            self._listener = None

    @property
    def connected(self) -> bool:
        return self._conn is not None

    # --- send / recv -------------------------------------------------------

    def send_sensor(self, frame: P.SensorFrame) -> None:
        assert self._conn is not None, "no client connected"
        wire = P.encode_frame(P.TYPE_SENSOR, frame.pack_payload())
        # Python's sendall already loops on partial sends, raises on error.
        self._conn.sendall(wire)
        self.stats.sensor_frames_sent += 1

    def recv_actuator(self, timeout: float | None = None) -> P.ActuatorFrame | None:
        """Pull bytes until one valid actuator frame is decoded.

        Returns None on timeout, raises ConnectionError if the peer closes.
        Garbage / CRC failures are absorbed by the decoder and tallied.
        """
        assert self._conn is not None, "no client connected"

        # Drain anything still in the decoder first.
        for f in self._decoder.drain():
            if f.frame_type == P.TYPE_ACTUATOR:
                self.stats.actuator_frames_received += 1
                return P.ActuatorFrame.unpack_payload(f.payload)

        self._conn.settimeout(timeout)
        try:
            while True:
                chunk = self._conn.recv(256)
                if not chunk:
                    raise ConnectionError("peer closed")
                self._decoder.feed(chunk)
                for f in self._decoder.drain():
                    if f.frame_type == P.TYPE_ACTUATOR:
                        self._merge_stats()
                        self.stats.actuator_frames_received += 1
                        return P.ActuatorFrame.unpack_payload(f.payload)
        except socket.timeout:
            return None
        finally:
            self._conn.settimeout(None)
            self._merge_stats()

    def _merge_stats(self) -> None:
        # FrameDecoder accumulates errors per-instance; mirror into LinkStats.
        self.stats.errors_crc = self._decoder.errors_crc
        self.stats.errors_size = self._decoder.errors_size
        self.stats.errors_type = self._decoder.errors_type
        self.stats.bytes_dropped = self._decoder.bytes_dropped


def now_ms() -> int:
    """Plant timebase — millisecond since process start. 32-bit wrap-safe."""
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF
