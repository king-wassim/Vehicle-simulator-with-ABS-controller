# Changelog

All notable changes to this project will be documented in this file.

Format inspired by [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
The project follows a 3-week sprint plan — each week ships as a tagged release.

---

## [Unreleased]

### Added
- `docs/JOURNAL.md` — day-by-day development journal (FR).
- `docs/SKILLS.md` — skills ↔ deliverables matrix mapped to KPIT/Vitesco job ads.
- `docs/INTERVIEW_QA.md` — 30 anticipated interview questions with prepared answers (FR).

### Planned (Week 3, days 15-21)
- `docs/FMEA.md` — formal failure-mode analysis (≥8 modes).
- `controller/src/app/diagnostic.{c,h}` — plausibility, stuck-sensor, noise detection.
- `plant/fault_injector.py` — CLI for injecting faults at runtime.
- `scripts/scenarios/` — 6 acceptance scenarios (nominal, ice patch, stuck sensor, comm loss, CRC, noise).
- `docs/RESULTS.md` — benchmarks (oracle vs C SIL, fault reaction times, jitter).
- README polish (architecture diagram, results plot, badges).

---

## [0.2.0] — 2026-05-27 — Week 2 — C ECU & real-time loop

### Added
- **HAL layer** (`controller/src/hal/`): `hal_sensors`, `hal_actuators`,
  `hal_link` (internal) so the app code is socket-agnostic — AUTOSAR-style.
- **State machine** (`controller/src/app/abs_state.{c,h}`):
  INIT → STANDBY → MONITOR → ACTIVE plus FAULT_DEGRADED / FAULT_LATCHED.
  Latch after 10 consecutive faults, sticky until reset.
- **Bang-bang controller** (`controller/src/app/abs_controller.{c,h}`):
  hysteresis around slip [0.10, 0.20], fail-operational pass-through in
  fault states, low-speed bypass below 5 km/h.
- **Real-time super-loop** (`controller/src/main.c`): 100 Hz via
  `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`, Welford-based jitter
  stats, CLI `host port duration driver_pedal_bar`.
- **Utils** (`controller/src/utils/time_utils.h`): timespec arithmetic +
  monotonic-ms helpers.
- **Unit tests**: 11 transitions in `test_abs_state.c`, 14 assertions in
  `test_abs_controller.c`. Total C unit tests: **30**.
- **W2 milestone**: `scripts/sil_brake_test.py` — full SIL co-simulation
  with stopping-distance assertions.
- **CI** extended with `make test` and the W2 SIL milestone.

### Measured
- Vehicle stops in **54.3 m** on dry asphalt under C control (envelope
  [35, 55] OK, comparable to Python oracle's 47.5 m).
- States visited: STANDBY → MONITOR → ACTIVE (no FAULT).
- DTC union: 0x0000.
- Jitter: **σ = 211 µs** over 410 cycles in WSL2 (target < 1 ms).

### Validated
- CI green on commit `a519c1b`:
  https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26503946600

---

## [0.1.0] — 2026-05-26 — Week 1 — Plant model, binary protocol, sockets

### Added
- **Documentation**: `docs/PHYSICS.md`, `docs/ARCHITECTURE.md`,
  `docs/PROTOCOL.md`.
- **Python plant**: `plant/tire.py` (Pacejka), `plant/vehicle.py`
  (quarter-car Newton + Euler), `plant/oracle_abs.py` (reference
  bang-bang), `plant/simulate.py` (scenarios + plots + envelope
  assertions), `plant/protocol.py` (wire layout + CRC + framing),
  `plant/socket_server.py` (one-client TCP server).
- **C drivers**: `controller/src/drivers/crc.{c,h}` (CRC-16/CCITT-FALSE),
  `protocol.h` (packed structs + `_Static_assert`), `socket_drv.{c,h}`
  (POSIX TCP with `TCP_NODELAY` and magic-resync).
- **Bilingual contract**: Python `protocol.py` and C `protocol.h` agree
  byte-for-byte (sensor 18 B, actuator 12 B).
- **Tests**: `tests/test_protocol.py` (13 Python: CRC KAT, layout,
  roundtrip, framing, resync, chunked feed, CRC mismatch, oversize),
  `controller/tests/test_crc.c` (5 C: KAT incl. cross-language anchor).
- **W1 milestone**: `scripts/milestone_w1.py` + `controller/tests/ping_client.c`
  — 1000-frame round-trip benchmark.
- **CI**: `.github/workflows/ci.yml` on ubuntu-22.04 — Python tests,
  plant envelope + plot artifact, C build with `-Werror`, C tests,
  W1 ping milestone with configurable latency threshold.
- **Build**: `controller/Makefile` compiles clean with
  `-std=c11 -Wall -Wextra -Wpedantic -Werror`.

### Measured
- `no_abs` braking: 59.6 m / 4.15 s (target ~60 m).
- `oracle` braking: 47.5 m / 3.56 s (target ~45 m).
- ABS gain: **20.3 %** distance saved.
- W1 milestone: 1000/1000 frames @ 1 kHz, **mean RTT 0.225 ms** (target
  < 1 ms), p99 0.669 ms, 0 CRC errors, 0 dropped bytes.

### Fixed
- Pacejka shape factor calibrated from 1.9 (lateral) to **1.65**
  (longitudinal) — matches the "locked wheel friction is ~65 % of peak"
  rule used in PHYSICS.md §3.
- Low-speed friction transition (`V_STATIC_FREEZE = 1.0 m/s`) so the
  vehicle actually stops instead of coasting on the zero-mu plateau.

### Validated
- CI green on commit `0526e91`:
  https://github.com/king-wassim/Vehicle-simulator-with-ABS-controller/actions/runs/26496884457

---

## [0.0.1] — 2026-05-26 — Project skeleton

### Added
- `README.md`, `docs/PHYSICS.md`, `docs/ARCHITECTURE.md` skeletons.
- Initial 21-day sprint plan and goals.
