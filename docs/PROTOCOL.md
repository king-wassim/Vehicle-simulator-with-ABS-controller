# PROTOCOL.md — Binary protocol between Plant (Python) and ECU (C)

> **Goal.** Specify the wire format so that the Python plant and the C
> controller agree byte-for-byte. Once frozen, this spec is the contract: any
> change must update both sides AND bump the protocol version.

---

## 1. Why a custom binary protocol (and not JSON / gRPC / etc.)

| Candidate | Verdict |
|---|---|
| JSON / msgpack | Too "PC-shaped". Adds tokenisation, dynamic sizing — does not exist on a real ECU. |
| Protobuf | Heavy, needs a generator, hides the framing problem. |
| **Custom packed struct** | ✅ Closest to what runs on real CAN/LIN/SPI buses. Forces us to handle endianness, alignment, framing, CRC ourselves — which is *exactly* the skill we want to prove. |

The protocol below imitates the typical layout of an OEM-defined CAN-payload
inside a tunnelling envelope (much like J1939 Transport Protocol or DoIP).

---

## 2. Topology and direction

```
              sensor_frame                            actuator_frame
   Plant ────────────────────────►   ECU   ────────────────────────►   Plant
   (Python, server)                  (C, client)
```

- **Plant is the TCP server** (binds, listens). Rationale: the plant is the
  "world" — it exists before the ECU boots, and an ECU may restart against a
  live world. This also matches the SIL convention used in CarMaker / VSL.
- **ECU is the TCP client** (connects on startup).
- Loopback only: `127.0.0.1:9000`.
- `TCP_NODELAY=1` on both sockets — disable Nagle to avoid the 40 ms
  aggregation delay on small frames.

---

## 3. Framing envelope

Every frame on the wire — regardless of direction — is wrapped in the same
envelope so that a receiver can resynchronise after dropped bytes:

```
+--------+--------+--------+--------+========+--------+--------+
| MAGIC  | TYPE   | LEN              | PAYLOAD| CRC16           |
| u16    | u8     | u16 (little-end) | ...    | u16             |
| 0xAA55 | 0x01/2 | payload bytes    |        | CRC-CCITT-FALSE |
+--------+--------+--------+--------+========+--------+--------+
   2B      1B            2B           N B           2B
```

| Field | Size | Description |
|---|---|---|
| `magic` | 2 B | Constant `0xAA55` (little-endian) — sync marker for resync. |
| `type` | 1 B | `0x01` = sensor_frame (P→E). `0x02` = actuator_frame (E→P). |
| `length` | 2 B | Number of bytes in the payload (does NOT include header or CRC). |
| `payload` | N B | The packed C struct (see §4). |
| `crc16` | 2 B | CRC-16/CCITT-FALSE computed over `type + length + payload`. |

Total overhead per frame: **7 bytes** (header) + **2 bytes** (CRC) = 9 bytes.

### 3.1 Endianness

**All multi-byte fields are little-endian.** Reasons:

1. x86_64 and most ARM Cortex-M run little-endian natively → no swap on either
   side, faster.
2. The packed struct can be `memcpy`'d directly into the payload with no
   per-field conversion.
3. Python uses `struct.pack('<...')` consistently.

### 3.2 Resynchronisation

A receiver MUST be able to recover from any number of dropped or corrupted
bytes (e.g. after a deserialisation bug or a flaky link). The recovery
algorithm:

```c
// pseudo
while (true) {
    read_until_magic_byte_pair(0xAA, 0x55);   // scan one byte at a time
    read_exact(header_remainder);             // type + length
    if (length > MAX_PAYLOAD) { drop; continue; }
    read_exact(payload, length);
    read_exact(crc, 2);
    if (crc16(type, length, payload) != crc) { drop; continue; }
    deliver(payload);
}
```

`MAX_PAYLOAD` is fixed at **64 bytes** — generous compared to our current
sizes (12 / 14 B) and guards against an attacker / faulty link sending
`length=0xFFFF`.

---

## 4. Payload definitions

### 4.1 `sensor_frame` (Plant → ECU, `type = 0x01`)

Mirrors what real wheel-speed sensors + a brake-pressure feedback line would
deliver to an ECU through CAN.

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;   //  4 B — plant clock at sample time
    float    v_vehicle;      //  4 B — m/s         (longitudinal speed)
    float    omega_wheel;    //  4 B — rad/s       (wheel angular speed)
    float    brake_pressure; //  4 B — bar         (feedback from hydraulic line)
    uint16_t fault_flags;    //  2 B — bitfield set by fault injector
} sensor_payload_t;          // 18 bytes
```

Endianness: little-endian. `float` is IEEE-754 binary32. The Python side
packs using `struct.pack('<I fff H', ...)`.

**`fault_flags` bitfield** (set by the Python fault injector — informative,
the ECU does NOT trust it for safety, only logs it):

| Bit | Name | Meaning |
|---|---|---|
| 0 | `FF_INJECTED_STUCK` | Sensor value frozen on purpose |
| 1 | `FF_INJECTED_NOISE` | Gaussian noise injected on omega |
| 2 | `FF_INJECTED_RANGE` | Out-of-range value injected |
| 3 | `FF_INJECTED_ICE` | Surface was hot-swapped to ice mid-run |
| 4..15 | reserved | must be 0 |

### 4.2 `actuator_frame` (ECU → Plant, `type = 0x02`)

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;   //  4 B — ECU clock at decision time
    float    brake_command;  //  4 B — bar (target line pressure)
    uint16_t ecu_status;     //  2 B — ABS state machine (see §4.3)
    uint16_t dtc_code;       //  2 B — Diagnostic Trouble Code bitfield (FMEA)
} actuator_payload_t;        // 12 bytes
```

### 4.3 `ecu_status` enumeration

| Value | Name | Meaning |
|---|---|---|
| 0 | `ECU_INIT` | Just booted, awaiting first valid frame |
| 1 | `ECU_STANDBY` | Vehicle stopped or below cut-in speed |
| 2 | `ECU_MONITOR` | Brake applied, slip within healthy band |
| 3 | `ECU_ACTIVE` | ABS modulating brake pressure |
| 4 | `ECU_FAULT_DEGRADED` | Fault detected; passing driver pedal through |
| 5 | `ECU_FAULT_LATCHED` | Repeated faults; locked in fault until reset |

### 4.4 `dtc_code` bitfield

Aligned with `docs/FMEA.md` (week 3). Stable subset:

| Bit | Name | Source mode |
|---|---|---|
| 0 | `DTC_SENSOR_STUCK` | F01 |
| 1 | `DTC_SENSOR_NOISE` | F02 |
| 2 | `DTC_COMM_TIMEOUT` | F03 |
| 3 | `DTC_COMM_CRC` | F04 |
| 4 | `DTC_SENSOR_RANGE` | F05 |
| 5 | `DTC_PLAUSIBILITY` | F06 |
| 6..15 | reserved | |

---

## 5. CRC-16 / CCITT-FALSE

We use **CRC-16/CCITT-FALSE** (a.k.a. CRC-16/AUTOSAR, polynomial `0x1021`)
because it is the one specified by AUTOSAR for end-to-end protection on CAN,
which is the closest real-world analogue to our scenario.

| Parameter | Value |
|---|---|
| Polynomial | `0x1021` |
| Initial value | `0xFFFF` |
| RefIn | false |
| RefOut | false |
| XorOut | `0x0000` |
| Check (`"123456789"`) | `0x29B1` |

The "check" vector is the canonical KAT — both the C and Python
implementations MUST reproduce `0x29B1` on the ASCII byte string `"123456789"`
in their unit test.

The CRC is computed over `type || length || payload` (the bytes between the
magic and the CRC field itself). Magic is excluded by design — if the magic
is corrupted, framing recovery handles it, not CRC.

---

## 6. Timing on the wire

- ECU period: **100 Hz** (every 10 ms). One `sensor_frame` consumed, one
  `actuator_frame` produced per cycle.
- Plant produces sensor frames at **100 Hz** too, lock-stepped to the same
  10 ms grid.
- Watchdog at the ECU: if no valid `sensor_frame` arrives within **50 ms**,
  the ECU enters `ECU_FAULT_DEGRADED` and passes the last known safe driver
  pedal through.

Note: in pure SIL, both ends share the same wall clock, but each side stamps
its OWN clock in `timestamp_ms` so that we can measure round-trip latency.

---

## 7. Frame size reference (compile-time check)

Both sides MUST `static_assert` (C) / `assert` (Python) these sizes:

| Frame | Total wire size |
|---|---|
| sensor frame (envelope + payload) | 9 + 18 = **27 B** |
| actuator frame (envelope + payload) | 9 + 12 = **21 B** |

At 100 Hz: ~2.7 KB/s up, ~2.1 KB/s down. Nowhere near saturation, even on
the slowest CAN bus.

---

## 8. Versioning

This document is **PROTOCOL v1**. Any breaking change (field added, type
renumbered, CRC algorithm swapped) bumps to v2 AND updates the magic to a
fresh constant (e.g. `0xAA56`) so that an old receiver cannot mistake a new
frame for a valid v1 one.
