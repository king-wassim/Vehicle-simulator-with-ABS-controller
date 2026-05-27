#include "abs_controller.h"

#include <math.h>

void abs_controller_init(abs_controller_ctx_t *ctx) {
    if (ctx == NULL) return;
    ctx->previous_command = 0.0f;
}

float abs_controller_slip(float v, float omega, float r) {
    /* Caller is responsible for v > 0 — see *_protected variant otherwise. */
    return (v - omega * r) / v;
}

float abs_controller_slip_protected(float v, float omega, float r, float v_min) {
    if (fabsf(v) < v_min) return 0.0f;
    float s = (v - omega * r) / v;
    if (s < 0.0f) return 0.0f;
    if (s > 1.0f) return 1.0f;
    return s;
}

float abs_controller_compute(abs_controller_ctx_t *ctx,
                             ecu_status_t          state,
                             const sensors_data_t *sensors,
                             float                 slip,
                             float                 driver_request) {
    if (ctx == NULL || sensors == NULL) return 0.0f;

    /* Fail-operational: hand the pedal back when we cannot trust the data. */
    if (state == ECU_FAULT_DEGRADED || state == ECU_FAULT_LATCHED) {
        ctx->previous_command = driver_request;
        return driver_request;
    }

    /* Low-speed bypass — slip is noisy here, hand over to the driver. */
    if (sensors->v_vehicle < ABS_LOW_SPEED_BYPASS_MPS) {
        ctx->previous_command = driver_request;
        return driver_request;
    }

    /* In INIT / STANDBY the driver request goes through unchanged — the
     * ABS only modulates once we're actively braking on a moving wheel. */
    if (state == ECU_INIT || state == ECU_STANDBY) {
        ctx->previous_command = driver_request;
        return driver_request;
    }

    /* Bang-bang with hysteresis (state == MONITOR or ACTIVE). */
    float cmd;
    if (slip > ABS_SLIP_UPPER)        cmd = 0.0f;            /* release */
    else if (slip < ABS_SLIP_LOWER)   cmd = driver_request;  /* re-apply */
    else                              cmd = ctx->previous_command;  /* hold */

    ctx->previous_command = cmd;
    return cmd;
}
