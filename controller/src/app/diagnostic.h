#ifndef ABS_DIAGNOSTIC_H
#define ABS_DIAGNOSTIC_H

/**
 * Diagnostic module — implements the failure-mode detectors that drive
 * the DTC bitfield. One concrete check per FMEA row (see docs/FMEA.md).
 *
 * Each detector is small, pure, and unit-tested in isolation, then
 * orchestrated by diagnostic_check() per cycle.
 *
 * Memory: zero malloc. Caller owns a `diagnostic_ctx_t` that holds the
 * sliding-window state needed by stuck and noise detection.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hal_sensors.h"
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- thresholds (also tested in test_diagnostic.c) ---- */

#define DIAG_STUCK_THRESHOLD_CYCLES   10u    /* 10 cycles @ 10 ms = 100 ms */
#define DIAG_STUCK_V_MIN              1.0f   /* m/s — only check stuck while the vehicle is meaningfully moving */
#define DIAG_NOISE_WINDOW_SIZE        20u    /* sliding window length */
/* Noise discrimination: random gaussian noise produces first-differences
 * that flip sign ~50 % of the time AND have non-trivial magnitude. A
 * controlled bang-bang transient also produces large deltas but with low
 * sign-flip rate (deltas keep the same sign during each ramp). Requiring
 * BOTH criteria together separates real noise from natural bang-bang. */
#define DIAG_NOISE_SIGN_CHANGES_MIN   12u    /* out of N-2 = 18 delta-pairs */
#define DIAG_NOISE_MEAN_ABS_DIFF_MIN  1.0f   /* rad/s — ignore tiny floating-point dust */
#define DIAG_OMEGA_MIN                (-1.0f)
#define DIAG_OMEGA_MAX                (500.0f)
#define DIAG_V_MIN                    (-1.0f)
#define DIAG_V_MAX                    (120.0f)     /* 432 km/h, generous */
#define DIAG_PLAUSIBILITY_EPSILON     (0.5f)       /* m/s tolerance on ωR > v */
#define DIAG_PLAUSIBILITY_CYCLES      5u
#define DIAG_CRC_ERROR_RATE_THRESHOLD 5u           /* errors per 100 cycles */

/* ---- per-instance context (caller-owned) ---- */

typedef struct {
    /* Stuck detector — omega frozen *while* v changes. */
    float    last_omega;
    float    last_v;
    uint32_t stuck_count;

    /* Noise detector — circular buffer of recent omega samples. We compute
     * variance of FIRST DIFFERENCES (delta[i] = omega[i] - omega[i-1]) so
     * that controlled bang-bang oscillations don't trigger the detector. */
    float    noise_window[DIAG_NOISE_WINDOW_SIZE];
    uint32_t noise_idx;
    bool     noise_window_full;

    /* Plausibility — count of consecutive ωR > v violations. */
    uint32_t plausibility_count;

    /* Comm error rate over a rolling window — simple decaying counter. */
    uint32_t crc_errors_in_window;
    uint32_t cycles_in_window;
    uint32_t last_crc_errors_seen;     /* monotonic snapshot from the link */
} diagnostic_ctx_t;

void diagnostic_init(diagnostic_ctx_t *ctx);

/**
 * Run every detector for the current cycle.
 *
 *   @param ctx                  per-instance state, mutated.
 *   @param sensors              current sample (may have .valid = false).
 *   @param wheel_radius_m       to compute the ωR check.
 *   @param cycles_since_valid   passed from main; drives the watchdog.
 *   @param crc_errors_total     monotonic count of CRC errors seen on the link.
 *
 * @return OR'ed DTC bitfield. DTC_NONE (0) if everything healthy.
 */
uint16_t diagnostic_check(diagnostic_ctx_t      *ctx,
                          const sensors_data_t  *sensors,
                          float                  wheel_radius_m,
                          uint32_t               cycles_since_valid,
                          uint32_t               crc_errors_total);

/* ---- individual detectors — public for unit tests --------------------- */

bool diagnostic_is_stuck(diagnostic_ctx_t *ctx, float omega_wheel, float v_vehicle);
bool diagnostic_is_noisy(diagnostic_ctx_t *ctx, float omega_wheel);
bool diagnostic_out_of_range(float v, float omega);
bool diagnostic_implausible(diagnostic_ctx_t *ctx, float v, float omega,
                            float wheel_radius_m);

#ifdef __cplusplus
}
#endif

#endif /* ABS_DIAGNOSTIC_H */
