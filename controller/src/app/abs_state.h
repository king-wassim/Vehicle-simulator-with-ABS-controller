#ifndef ABS_STATE_H
#define ABS_STATE_H

/**
 * ABS state machine — high-level operating mode of the ECU.
 *
 *                ┌──────────┐
 *                │   INIT   │   (entry — no data yet)
 *                └────┬─────┘   first valid frame
 *                     ▼
 *   ┌─────────►  STANDBY  ◄──── v < 5 km/h (low-speed bypass)
 *   │            └───┬──────┘
 *   │                │ brake_pressure > BRAKE_ACTIVE_THRESHOLD
 *   │                ▼
 *   │            MONITOR  ────► slip > UPPER  ─────► ACTIVE
 *   │            ▲     ▲                                 │
 *   │            │     └─── slip < LOWER  ──────────────┘
 *   │            │
 *   │ all-clear: │
 *   │            │
 *   │       FAULT_DEGRADED  ◄── DTC raised (from any state)
 *   │            │
 *   │            ▼
 *   └──── FAULT_LATCHED  ◄── N consecutive faults; only a reset clears it.
 *
 * Mapping to wire enum `ecu_status_t` in protocol.h is 1:1 so the same
 * uint16_t can be reported to the plant without translation.
 */

#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"      /* ecu_status_t */
#include "hal_sensors.h"   /* sensors_data_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Thresholds — public so unit tests can pin the contract. */
#define ABS_LOW_SPEED_BYPASS_MPS    (5.0f / 3.6f)   /* 5 km/h */
#define ABS_BRAKE_ACTIVE_BAR        (5.0f)          /* driver "really braking" */
#define ABS_SLIP_UPPER              (0.20f)
#define ABS_SLIP_LOWER              (0.10f)
#define ABS_FAULT_LATCH_THRESHOLD   (10u)           /* consecutive faults */

/** Opaque context — caller owns the struct, module never allocates. */
typedef struct {
    ecu_status_t current;
    ecu_status_t previous;
    uint32_t     fault_count;        /* consecutive cycles with a DTC */
    uint32_t     cycles_in_state;    /* counter, useful for telemetry */
} abs_state_ctx_t;

/** Zero out the context — must be called once before abs_state_update(). */
void abs_state_init(abs_state_ctx_t *ctx);

/**
 * Step the state machine one cycle.
 *
 *   @param sensors   latest sensor sample (valid==false handled as comm fault)
 *   @param slip      slip ratio from abs_controller_compute_slip()
 *   @param dtc       OR'ed DTC bitfield from the diagnostic module
 *
 * Returns the new current state, also written into `ctx->current`. The
 * function is pure with respect to `ctx` — same inputs, same outputs.
 */
ecu_status_t abs_state_update(abs_state_ctx_t      *ctx,
                              const sensors_data_t *sensors,
                              float                 slip,
                              uint16_t              dtc);

/** Human-readable label for logging / diagnostics. */
const char *abs_state_name(ecu_status_t s);

/** Did the state actually transition this last update? */
static inline bool abs_state_changed(const abs_state_ctx_t *ctx) {
    return ctx->current != ctx->previous;
}

#ifdef __cplusplus
}
#endif

#endif /* ABS_STATE_H */
