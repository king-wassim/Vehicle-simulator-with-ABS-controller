#ifndef ABS_PROTOCOL_H
#define ABS_PROTOCOL_H

/**
 * Wire protocol — see docs/PROTOCOL.md for the authoritative spec.
 *
 * Both sides agree on:
 *   - little-endian on every multi-byte field
 *   - IEEE-754 binary32 for float
 *   - CRC-16/CCITT-FALSE over (type || length || payload)
 *
 * Sizes are static_asserted below so a mismatched compiler will fail at
 * build time, not at 03:00 on a freezing test bench.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_MAGIC          0xAA55u
#define PROTO_TYPE_SENSOR    0x01u
#define PROTO_TYPE_ACTUATOR  0x02u
#define PROTO_MAX_PAYLOAD    64u

/* Frame envelope: magic(2) + type(1) + len(2) + payload(N) + crc(2). */
#define PROTO_HEADER_SIZE    5u
#define PROTO_CRC_SIZE       2u
#define PROTO_OVERHEAD       (PROTO_HEADER_SIZE + PROTO_CRC_SIZE)

/* ---- sensor_payload_t (Plant -> ECU) ---- */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    v_vehicle;       /* m/s   */
    float    omega_wheel;     /* rad/s */
    float    brake_pressure;  /* bar   */
    uint16_t fault_flags;     /* informational bitfield from injector */
} sensor_payload_t;

/* fault_flags bits (informative — ECU logs them but never trusts them) */
#define FF_INJECTED_STUCK   (1u << 0)
#define FF_INJECTED_NOISE   (1u << 1)
#define FF_INJECTED_RANGE   (1u << 2)
#define FF_INJECTED_ICE     (1u << 3)

/* ---- actuator_payload_t (ECU -> Plant) ---- */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float    brake_command;   /* bar — target line pressure */
    uint16_t ecu_status;      /* see ECU_* below */
    uint16_t dtc_code;        /* bitfield, see DTC_* below */
} actuator_payload_t;

/* ECU state machine — wire values are stable. */
typedef enum {
    ECU_INIT             = 0,
    ECU_STANDBY          = 1,
    ECU_MONITOR          = 2,
    ECU_ACTIVE           = 3,
    ECU_FAULT_DEGRADED   = 4,
    ECU_FAULT_LATCHED    = 5,
} ecu_status_t;

/* Diagnostic trouble codes — keep in sync with docs/FMEA.md. */
#define DTC_NONE            0x0000u
#define DTC_SENSOR_STUCK    (1u << 0)   /* F01 */
#define DTC_SENSOR_NOISE    (1u << 1)   /* F02 */
#define DTC_COMM_TIMEOUT    (1u << 2)   /* F03 */
#define DTC_COMM_CRC        (1u << 3)   /* F04 */
#define DTC_SENSOR_RANGE    (1u << 4)   /* F05 */
#define DTC_PLAUSIBILITY    (1u << 5)   /* F06 */

/* Compile-time guards — payload sizes are part of the contract. */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(sensor_payload_t)   == 18, "sensor_payload_t size drifted");
_Static_assert(sizeof(actuator_payload_t) == 12, "actuator_payload_t size drifted");
#endif

#ifdef __cplusplus
}
#endif

#endif /* ABS_PROTOCOL_H */
