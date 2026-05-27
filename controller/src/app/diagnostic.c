#include "diagnostic.h"

#include <math.h>
#include <string.h>

#define WATCHDOG_TIMEOUT_CYCLES   5u     /* 50 ms / 10 ms */
#define CRC_RATE_DENOMINATOR      100u   /* errors / 100 cycles */

/* ---- init ------------------------------------------------------------ */

void diagnostic_init(diagnostic_ctx_t *ctx) {
    if (ctx == NULL) return;
    memset(ctx, 0, sizeof(*ctx));
}

/* ---- F01: stuck detector --------------------------------------------- */

bool diagnostic_is_stuck(diagnostic_ctx_t *ctx, float omega, float v_vehicle) {
    /* "stuck" = omega is bit-exactly identical for N consecutive cycles
     * while the vehicle is meaningfully moving. Rationale:
     *   - In healthy operation, omega comes from a physics computation that
     *     is essentially never bit-identical between cycles (floating-point
     *     dust + controller updates) — so exact equality is a strong signal.
     *   - At standstill we explicitly skip the check, because omega legitimately
     *     stays at 0.0 when the vehicle is parked.
     *   - When the controller releases the brake on a degraded run, the
     *     wheel re-accelerates from road friction — so omega would still
     *     evolve unless the SENSOR itself is hung. */
    bool omega_frozen   = (omega == ctx->last_omega);
    bool vehicle_moving = v_vehicle > DIAG_STUCK_V_MIN;

    if (omega_frozen && vehicle_moving) {
        ctx->stuck_count++;
    } else {
        ctx->stuck_count = 0;
    }
    ctx->last_omega = omega;
    ctx->last_v     = v_vehicle;
    return ctx->stuck_count >= DIAG_STUCK_THRESHOLD_CYCLES;
}

/* ---- F02: noise detector (sliding variance) -------------------------- */

bool diagnostic_is_noisy(diagnostic_ctx_t *ctx, float omega) {
    ctx->noise_window[ctx->noise_idx] = omega;
    ctx->noise_idx = (ctx->noise_idx + 1u) % DIAG_NOISE_WINDOW_SIZE;
    if (ctx->noise_idx == 0u) ctx->noise_window_full = true;

    if (!ctx->noise_window_full) return false;   /* not enough data yet */

    /* Reassemble the window in chronological order (oldest first) so
     * first-differences are computed correctly. */
    float win[DIAG_NOISE_WINDOW_SIZE];
    for (uint32_t k = 0; k < DIAG_NOISE_WINDOW_SIZE; ++k) {
        win[k] = ctx->noise_window[(ctx->noise_idx + k) % DIAG_NOISE_WINDOW_SIZE];
    }

    /* Combined noise detector. δ[i] = ω[i+1] − ω[i].
     *   - sign_changes: number of pairs (δ[i-1], δ[i]) where the sign
     *     flipped. Pure ramp → 0. Bang-bang → small (one per transition).
     *     White noise → ~50 % of pairs, i.e. ~9 out of 18.
     *   - mean_abs_d: average absolute delta. Filters out near-zero floating-
     *     point jitter that would otherwise trip the sign-flip counter.
     * Both criteria must hold for the detector to fire. */
    const uint32_t N = DIAG_NOISE_WINDOW_SIZE;
    uint32_t sign_changes = 0;
    float    abs_sum      = 0.0f;
    float    prev_delta   = win[1] - win[0];
    abs_sum += fabsf(prev_delta);
    for (uint32_t i = 2; i < N; ++i) {
        float delta = win[i] - win[i - 1];
        abs_sum += fabsf(delta);
        if (prev_delta * delta < 0.0f) sign_changes++;
        prev_delta = delta;
    }
    float mean_abs_d = abs_sum / (float)(N - 1u);

    return sign_changes  >= DIAG_NOISE_SIGN_CHANGES_MIN
        && mean_abs_d    >= DIAG_NOISE_MEAN_ABS_DIFF_MIN;
}

/* ---- F05: range check ------------------------------------------------ */

bool diagnostic_out_of_range(float v, float omega) {
    if (omega < DIAG_OMEGA_MIN || omega > DIAG_OMEGA_MAX) return true;
    if (v     < DIAG_V_MIN     || v     > DIAG_V_MAX)     return true;
    /* Also catch NaN / Inf — a single denormal in the chain pollutes
     * everything downstream. */
    if (!isfinite(omega) || !isfinite(v)) return true;
    return false;
}

/* ---- F07: physical plausibility (ωR > v in braking) ------------------ */

bool diagnostic_implausible(diagnostic_ctx_t *ctx, float v, float omega,
                            float wheel_radius_m) {
    /* Negative slip = wheel spins faster than the body. Physically
     * impossible under pure braking — would mean the wheel is being
     * driven by the road (which only happens on traction, not braking). */
    if (v < 1.0f) {
        /* At standstill / crawl, this check is meaningless. */
        ctx->plausibility_count = 0;
        return false;
    }
    if (omega * wheel_radius_m > v + DIAG_PLAUSIBILITY_EPSILON) {
        ctx->plausibility_count++;
    } else {
        ctx->plausibility_count = 0;
    }
    return ctx->plausibility_count >= DIAG_PLAUSIBILITY_CYCLES;
}

/* ---- orchestrator ---------------------------------------------------- */

uint16_t diagnostic_check(diagnostic_ctx_t     *ctx,
                          const sensors_data_t *sensors,
                          float                 wheel_radius_m,
                          uint32_t              cycles_since_valid,
                          uint32_t              crc_errors_total) {
    if (ctx == NULL || sensors == NULL) return DTC_PLAUSIBILITY;

    uint16_t dtc = DTC_NONE;

    /* F03: comm timeout — independent of sensor validity. */
    if (cycles_since_valid > WATCHDOG_TIMEOUT_CYCLES) {
        dtc |= DTC_COMM_TIMEOUT;
    }

    /* F04: CRC error rate. Maintain a rolling 100-cycle window of how
     * many CRC errors have appeared. Above the threshold → DTC. */
    ctx->cycles_in_window++;
    if (ctx->cycles_in_window >= CRC_RATE_DENOMINATOR) {
        if (ctx->crc_errors_in_window >= DIAG_CRC_ERROR_RATE_THRESHOLD) {
            dtc |= DTC_COMM_CRC;
        }
        ctx->crc_errors_in_window = 0;
        ctx->cycles_in_window = 0;
    }
    if (crc_errors_total > ctx->last_crc_errors_seen) {
        ctx->crc_errors_in_window += (crc_errors_total - ctx->last_crc_errors_seen);
        ctx->last_crc_errors_seen = crc_errors_total;
    }

    /* If the sample is not valid (sensor not refreshed this cycle), we
     * can still report the comm DTC but skip the sensor-content checks. */
    if (!sensors->valid) {
        return dtc;
    }

    /* F05: range. */
    if (diagnostic_out_of_range(sensors->v_vehicle, sensors->omega_wheel)) {
        dtc |= DTC_SENSOR_RANGE;
        /* On an aberrant value, skip the other content checks — they'd
         * misfire on garbage. */
        return dtc;
    }

    /* F01: stuck. */
    if (diagnostic_is_stuck(ctx, sensors->omega_wheel, sensors->v_vehicle)) {
        dtc |= DTC_SENSOR_STUCK;
    }

    /* F02: noise. Disabled in the orchestrator for now — natural bang-bang
     * transients produce sign-change patterns that overlap with what a
     * mildly noisy sensor would produce, so simple statistical detectors
     * misfire during ACTIVE. The detector function itself is correct and
     * exercised in test_diagnostic.c; the right call is to characterize a
     * real noise injection (see scripts/scenarios/noisy_sensor.py, W3 J18-19)
     * and either re-tune the threshold or gate noise checks on the ECU
     * state machine. */
#if 0
    if (diagnostic_is_noisy(ctx, sensors->omega_wheel)) {
        dtc |= DTC_SENSOR_NOISE;
    }
#endif

    /* F06 / F07: plausibility. */
    if (diagnostic_implausible(ctx, sensors->v_vehicle, sensors->omega_wheel,
                                wheel_radius_m)) {
        dtc |= DTC_PLAUSIBILITY;
    }

    return dtc;
}
