#include "abs_state.h"

#include <string.h>

/* ---- helpers ---------------------------------------------------------- */

static bool sensors_usable(const sensors_data_t *s) {
    if (s == NULL || !s->valid) return false;
    /* Range sanity — anything stranger than this means upstream is broken
     * and we should not derive a control decision from it. The diagnostic
     * module also raises DTCs for these, but we double-check here so the
     * state machine is robust on its own. */
    if (s->v_vehicle < -1.0f || s->v_vehicle > 120.0f)     return false;
    if (s->omega_wheel < -1.0f || s->omega_wheel > 500.0f) return false;
    return true;
}

static bool vehicle_stopped(const sensors_data_t *s) {
    return s->v_vehicle < ABS_LOW_SPEED_BYPASS_MPS;
}

static bool driver_braking(const sensors_data_t *s) {
    return s->brake_pressure > ABS_BRAKE_ACTIVE_BAR;
}

/* ---- public API ------------------------------------------------------- */

void abs_state_init(abs_state_ctx_t *ctx) {
    if (ctx == NULL) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->current  = ECU_INIT;
    ctx->previous = ECU_INIT;
}

ecu_status_t abs_state_update(abs_state_ctx_t      *ctx,
                              const sensors_data_t *sensors,
                              float                 slip,
                              uint16_t              dtc) {
    if (ctx == NULL) return ECU_FAULT_LATCHED;

    ctx->previous = ctx->current;

    /* ---- 1. fault handling has absolute priority ---------------------- */
    if (dtc != DTC_NONE || !sensors_usable(sensors)) {
        ctx->fault_count++;
        if (ctx->fault_count >= ABS_FAULT_LATCH_THRESHOLD) {
            ctx->current = ECU_FAULT_LATCHED;
        } else if (ctx->current != ECU_FAULT_LATCHED) {
            ctx->current = ECU_FAULT_DEGRADED;
        }
        ctx->cycles_in_state = abs_state_changed(ctx) ? 0u : ctx->cycles_in_state + 1u;
        return ctx->current;
    }

    /* Once latched, only an explicit init can clear it. */
    if (ctx->current == ECU_FAULT_LATCHED) {
        ctx->cycles_in_state++;
        return ctx->current;
    }

    /* Healthy sample — decay the fault counter. */
    ctx->fault_count = 0;

    /* ---- 2. normal-mode transitions ----------------------------------- */
    switch (ctx->current) {
    case ECU_INIT:
        /* First healthy sample takes us into operational space. */
        ctx->current = vehicle_stopped(sensors) ? ECU_STANDBY : ECU_MONITOR;
        break;

    case ECU_STANDBY:
        if (!vehicle_stopped(sensors) && driver_braking(sensors)) {
            ctx->current = ECU_MONITOR;
        }
        break;

    case ECU_MONITOR:
        if (vehicle_stopped(sensors)) {
            ctx->current = ECU_STANDBY;
        } else if (slip > ABS_SLIP_UPPER) {
            ctx->current = ECU_ACTIVE;
        }
        /* If the driver releases the pedal, stay in MONITOR — we are still
         * watching slip in case they re-apply quickly. The plant simulation
         * leaves brake_pressure at 0 at that point, which is fine. */
        break;

    case ECU_ACTIVE:
        if (vehicle_stopped(sensors)) {
            ctx->current = ECU_STANDBY;
        } else if (slip < ABS_SLIP_LOWER) {
            /* Slip is back inside the safe zone — return to MONITOR. The
             * controller will keep modulating with hysteresis. */
            ctx->current = ECU_MONITOR;
        }
        break;

    case ECU_FAULT_DEGRADED:
        /* Recovered. Re-evaluate from a fresh STANDBY/MONITOR. */
        ctx->current = vehicle_stopped(sensors) ? ECU_STANDBY : ECU_MONITOR;
        break;

    case ECU_FAULT_LATCHED:
    default:
        /* unreachable here (handled above) */
        break;
    }

    ctx->cycles_in_state = abs_state_changed(ctx) ? 0u : ctx->cycles_in_state + 1u;
    return ctx->current;
}

const char *abs_state_name(ecu_status_t s) {
    switch (s) {
    case ECU_INIT:             return "INIT";
    case ECU_STANDBY:          return "STANDBY";
    case ECU_MONITOR:          return "MONITOR";
    case ECU_ACTIVE:           return "ACTIVE";
    case ECU_FAULT_DEGRADED:   return "FAULT_DEGRADED";
    case ECU_FAULT_LATCHED:    return "FAULT_LATCHED";
    default:                   return "?";
    }
}
