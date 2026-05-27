"""End-of-week-1 milestone test.

Pushes N sensor frames at the configured rate to the C ping_client, expects
N actuator frames back where brake_command == v_vehicle (echo). Measures
round-trip latency, asserts:

    * 0 CRC errors
    * 0 dropped frames
    * mean round-trip latency < 1 ms (loopback, TCP_NODELAY)

Usage (in two terminals):
    # T1 (build then run the client — exits after N frames)
    make -C controller ping_client
    ./controller/build/ping_client 127.0.0.1 9000 1000

    # T2 (run this script — start it first so the client has someone to connect to)
    python -m scripts.milestone_w1 --frames 1000
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

from plant import protocol as P
from plant.socket_server import PlantLink, now_ms


def run(frames: int, rate_hz: float, port: int, host: str,
        max_mean_latency_ms: float) -> int:
    link = PlantLink(host, port)
    link.listen()
    print(f"[milestone] listening on {host}:{port}, waiting for client...")
    link.accept()
    print("[milestone] client connected.")

    period = 1.0 / rate_hz
    latencies: list[float] = []
    mismatches = 0

    t_start = time.perf_counter()
    next_tick = t_start
    for i in range(frames):
        v = 20.0 + 0.001 * i        # vary so the echo can spot a desync
        omega = v / 0.30
        s = P.SensorFrame(timestamp_ms=now_ms(),
                          v_vehicle=v, omega_wheel=omega,
                          brake_pressure=0.0, fault_flags=0)
        t_send = time.perf_counter()
        link.send_sensor(s)

        a = link.recv_actuator(timeout=2.0)
        t_recv = time.perf_counter()

        if a is None:
            print(f"[milestone] FAIL: timeout at frame {i}", file=sys.stderr)
            return 1
        if abs(a.brake_command - v) > 1e-3:
            mismatches += 1

        latencies.append((t_recv - t_send) * 1000.0)

        next_tick += period
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)

    elapsed = time.perf_counter() - t_start
    link.close()

    mean = statistics.fmean(latencies)
    p50  = statistics.median(latencies)
    p95  = statistics.quantiles(latencies, n=20)[18]
    p99  = statistics.quantiles(latencies, n=100)[98]

    print(f"\n=== milestone results (loopback, TCP_NODELAY) ===")
    print(f"frames sent / acked     : {frames} / {frames - mismatches}")
    print(f"echo mismatches         : {mismatches}")
    print(f"elapsed                 : {elapsed*1000:.1f} ms  ({frames/elapsed:.0f} Hz)")
    print(f"round-trip latency [ms] : mean={mean:.3f}  p50={p50:.3f}  p95={p95:.3f}  p99={p99:.3f}")
    print(f"server-side decode errs : crc={link.stats.errors_crc}  "
          f"size={link.stats.errors_size}  type={link.stats.errors_type}  "
          f"dropped_bytes={link.stats.bytes_dropped}")

    crit = []
    if mismatches > 0:            crit.append("echo mismatches > 0")
    if link.stats.errors_crc > 0: crit.append("server CRC errors > 0")
    if mean > max_mean_latency_ms:
        crit.append(f"mean latency {mean:.3f} ms > {max_mean_latency_ms} ms target")
    if crit:
        for c in crit: print(f"  FAIL: {c}", file=sys.stderr)
        return 1
    print("\n[milestone] OK")
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--frames", type=int, default=1000)
    p.add_argument("--rate", type=float, default=100.0,
                   help="target send rate in Hz (default 100)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--max-mean-latency-ms", type=float, default=1.0,
                   help="fail if mean RTT exceeds this many ms "
                        "(loosen on noisy CI runners)")
    args = p.parse_args(argv)
    return run(args.frames, args.rate, args.port, args.host,
               args.max_mean_latency_ms)


if __name__ == "__main__":
    sys.exit(main())
