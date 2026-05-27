/**
 * Diagnostic module unit tests — one test per FMEA detector + the
 * orchestrator combining several at once.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diagnostic.h"

static int failed = 0;

#define EXPECT_TRUE(cond, label)                                             \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s: expected true\n", (label));            \
            failed++;                                                        \
        } else {                                                             \
            printf("  ok  %s\n", (label));                                   \
        }                                                                    \
    } while (0)

#define EXPECT_FALSE(cond, label)                                            \
    do {                                                                     \
        if ((cond)) {                                                        \
            fprintf(stderr, "FAIL %s: expected false\n", (label));           \
            failed++;                                                        \
        } else {                                                             \
            printf("  ok  %s (false)\n", (label));                           \
        }                                                                    \
    } while (0)

#define EXPECT_EQ_HEX(actual, expected, label)                               \
    do {                                                                     \
        uint16_t a = (uint16_t)(actual), e = (uint16_t)(expected);           \
        if (a != e) {                                                        \
            fprintf(stderr, "FAIL %s: got 0x%04X, want 0x%04X\n",            \
                    (label), a, e);                                          \
            failed++;                                                        \
        } else {                                                             \
            printf("  ok  %s = 0x%04X\n", (label), a);                       \
        }                                                                    \
    } while (0)

static sensors_data_t mk(float v, float omega) {
    sensors_data_t s = {0};
    s.valid = true;
    s.v_vehicle = v;
    s.omega_wheel = omega;
    return s;
}

/* ---- F01 stuck ----------------------------------------------------- */

static void test_stuck_after_threshold(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool stuck = false;
    /* omega frozen at 42 while vehicle is moving — sensor decoupled from physics. */
    for (unsigned i = 0; i < DIAG_STUCK_THRESHOLD_CYCLES + 2; ++i) {
        stuck = diagnostic_is_stuck(&c, 42.0f, 20.0f);
    }
    EXPECT_TRUE(stuck, "stuck detected after threshold (omega frozen, vehicle moving)");
}

static void test_stuck_resets_on_omega_change(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    for (unsigned i = 0; i < 5; ++i)
        (void)diagnostic_is_stuck(&c, 42.0f, 20.0f);
    EXPECT_FALSE(diagnostic_is_stuck(&c, 42.001f, 20.0f),
                 "stuck count resets when omega moves");
}

static void test_stuck_ignored_when_vehicle_stopped(void) {
    /* Vehicle parked: omega = 0 forever — NOT stuck, just standing still. */
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool stuck = false;
    for (unsigned i = 0; i < DIAG_STUCK_THRESHOLD_CYCLES * 2; ++i) {
        stuck = diagnostic_is_stuck(&c, 0.0f, 0.0f);
    }
    EXPECT_FALSE(stuck, "no false-positive when vehicle is parked");
}

static void test_stuck_below_threshold_does_not_fire(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool stuck = false;
    for (unsigned i = 0; i < DIAG_STUCK_THRESHOLD_CYCLES - 1; ++i) {
        stuck = diagnostic_is_stuck(&c, 30.0f, 15.0f);
    }
    EXPECT_FALSE(stuck, "below threshold: not yet stuck");
}

/* ---- F02 noise ----------------------------------------------------- */

static void test_noise_quiet_signal(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool noisy = false;
    /* Slowly varying signal — first-differences are tiny, variance ≈ 0. */
    for (unsigned i = 0; i < DIAG_NOISE_WINDOW_SIZE * 2; ++i) {
        noisy = diagnostic_is_noisy(&c, 50.0f + 0.01f * (float)i);
    }
    EXPECT_FALSE(noisy, "smooth signal is not flagged as noisy");
}

static void test_noise_ignores_bangbang_oscillation(void) {
    /* Bang-bang in real SIL: omega ramps smoothly because the wheel inertia
     * limits dω/dt. We mimic this with a smooth triangle wave of ±2 rad/s
     * per cycle — first-differences are nearly constant -> low diff-variance. */
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool noisy = false;
    for (unsigned i = 0; i < DIAG_NOISE_WINDOW_SIZE * 4; ++i) {
        unsigned phase = i % 20;
        float omega = (phase < 10) ? (40.0f + 2.0f * (float)phase)
                                   : (60.0f - 2.0f * (float)(phase - 10));
        noisy = diagnostic_is_noisy(&c, omega);
    }
    EXPECT_FALSE(noisy, "controlled oscillation is not noise (first-diff variance)");
}

static void test_noise_detects_high_freq_jitter(void) {
    /* Square wave at sample rate (alternates every cycle) — every first-
     * difference is ±10 rad/s -> var of diffs ≈ 400. Way above threshold. */
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool noisy = false;
    for (unsigned i = 0; i < DIAG_NOISE_WINDOW_SIZE * 2; ++i) {
        float omega = (i & 1) ? 55.0f : 45.0f;
        noisy = diagnostic_is_noisy(&c, omega);
    }
    EXPECT_TRUE(noisy, "alternating ±10 rad/s flagged as noise");
}

/* ---- F05 range ----------------------------------------------------- */

static void test_range_rejects_negative_omega(void) {
    EXPECT_TRUE(diagnostic_out_of_range(20.0f, -10.0f), "omega < 0 rejected");
}

static void test_range_rejects_huge_v(void) {
    EXPECT_TRUE(diagnostic_out_of_range(200.0f, 50.0f), "v=200 m/s rejected");
}

static void test_range_rejects_nan(void) {
    EXPECT_TRUE(diagnostic_out_of_range(20.0f, NAN), "NaN omega rejected");
}

static void test_range_accepts_nominal(void) {
    EXPECT_FALSE(diagnostic_out_of_range(20.0f, 66.0f),
                 "nominal (v=20, ω=66) accepted");
}

/* ---- F07 implausibility -------------------------------------------- */

static void test_plausibility_detects_omega_gt_v(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    bool impl = false;
    /* v=20, ω*R = 25 -> impossible in pure braking. Need N consecutive. */
    for (unsigned i = 0; i < DIAG_PLAUSIBILITY_CYCLES + 1; ++i) {
        impl = diagnostic_implausible(&c, 20.0f, 25.0f / 0.30f, 0.30f);
    }
    EXPECT_TRUE(impl, "ωR > v for N cycles flagged implausible");
}

static void test_plausibility_resets_on_normal(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    for (unsigned i = 0; i < DIAG_PLAUSIBILITY_CYCLES + 1; ++i)
        (void)diagnostic_implausible(&c, 20.0f, 25.0f / 0.30f, 0.30f);
    EXPECT_FALSE(diagnostic_implausible(&c, 20.0f, 18.0f / 0.30f, 0.30f),
                 "plausibility resets when ωR <= v");
}

/* ---- orchestrator -------------------------------------------------- */

static void test_orchestrator_healthy_returns_none(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    sensors_data_t s = mk(20.0f, 66.0f);
    uint16_t dtc = diagnostic_check(&c, &s, 0.30f, 0, 0);
    EXPECT_EQ_HEX(dtc, DTC_NONE, "healthy sample -> no DTC");
}

static void test_orchestrator_invalid_sensor_only_comm(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    sensors_data_t s = mk(20.0f, 66.0f);
    s.valid = false;
    /* cycles_since_valid above watchdog -> COMM_TIMEOUT only, no content checks. */
    uint16_t dtc = diagnostic_check(&c, &s, 0.30f, 10, 0);
    EXPECT_EQ_HEX(dtc, DTC_COMM_TIMEOUT, "invalid + watchdog -> COMM_TIMEOUT");
}

static void test_orchestrator_range_short_circuits(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    sensors_data_t s = mk(20.0f, -50.0f);   /* out of range */
    uint16_t dtc = diagnostic_check(&c, &s, 0.30f, 0, 0);
    EXPECT_EQ_HEX(dtc, DTC_SENSOR_RANGE,
                  "range error short-circuits other checks");
}

static void test_orchestrator_stuck_after_run(void) {
    diagnostic_ctx_t c; diagnostic_init(&c);
    uint16_t dtc = 0;
    /* omega frozen at 66 while v decreases — sensor decoupled from physics. */
    for (unsigned i = 0; i < DIAG_STUCK_THRESHOLD_CYCLES + 2; ++i) {
        sensors_data_t s = mk(20.0f - 0.5f * (float)i, 66.0f);
        dtc = diagnostic_check(&c, &s, 0.30f, 0, 0);
    }
    EXPECT_TRUE((dtc & DTC_SENSOR_STUCK) != 0,
                "stuck DTC raised after threshold via orchestrator");
}

int main(void) {
    /* F01 stuck */
    test_stuck_after_threshold();
    test_stuck_resets_on_omega_change();
    test_stuck_ignored_when_vehicle_stopped();
    test_stuck_below_threshold_does_not_fire();
    /* F02 noise */
    test_noise_quiet_signal();
    test_noise_ignores_bangbang_oscillation();
    test_noise_detects_high_freq_jitter();
    /* F05 range */
    test_range_rejects_negative_omega();
    test_range_rejects_huge_v();
    test_range_rejects_nan();
    test_range_accepts_nominal();
    /* F07 implausibility */
    test_plausibility_detects_omega_gt_v();
    test_plausibility_resets_on_normal();
    /* orchestrator */
    test_orchestrator_healthy_returns_none();
    test_orchestrator_invalid_sensor_only_comm();
    test_orchestrator_range_short_circuits();
    test_orchestrator_stuck_after_run();

    if (failed) {
        fprintf(stderr, "\n%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    printf("\nall diagnostic tests passed.\n");
    return EXIT_SUCCESS;
}
