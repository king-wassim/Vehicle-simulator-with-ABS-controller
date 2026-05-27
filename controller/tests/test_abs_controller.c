/**
 * abs_controller unit tests — slip math + bang-bang logic + fail-operational.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "abs_controller.h"

static int failed = 0;

#define EXPECT_NEAR(actual, expected, tol, label)                            \
    do {                                                                     \
        float a = (float)(actual), e = (float)(expected);                    \
        if (fabsf(a - e) > (tol)) {                                          \
            fprintf(stderr, "FAIL %s: got %.6f, want %.6f (tol %.6f)\n",     \
                    (label), a, e, (float)(tol));                            \
            failed++;                                                        \
        } else {                                                             \
            printf("  ok  %s = %.4f\n", (label), a);                         \
        }                                                                    \
    } while (0)

static sensors_data_t mk(float v, float omega, float pbar) {
    sensors_data_t s = {0};
    s.valid = true;
    s.v_vehicle = v;
    s.omega_wheel = omega;
    s.brake_pressure = pbar;
    return s;
}

/* ---- slip math ---- */

static void test_slip_free_wheel(void) {
    /* v = omega * R exactly → slip = 0 */
    float s = abs_controller_slip(20.0f, 20.0f / 0.30f, 0.30f);
    EXPECT_NEAR(s, 0.0f, 1e-5f, "slip(free rolling) = 0");
}

static void test_slip_locked_wheel(void) {
    /* omega = 0 → slip = 1 */
    float s = abs_controller_slip(20.0f, 0.0f, 0.30f);
    EXPECT_NEAR(s, 1.0f, 1e-5f, "slip(locked) = 1");
}

static void test_slip_protected_at_zero_speed(void) {
    /* F06: must not divide by zero at standstill. */
    float s = abs_controller_slip_protected(0.5f, 0.0f, 0.30f, 1.0f);
    EXPECT_NEAR(s, 0.0f, 1e-6f, "slip protected at v < v_min");
}

static void test_slip_protected_clamps_to_unit_interval(void) {
    float s_neg = abs_controller_slip_protected(20.0f, 80.0f, 0.30f, 1.0f);
    EXPECT_NEAR(s_neg, 0.0f, 1e-6f, "negative slip clamped to 0");
    float s_big = abs_controller_slip_protected(20.0f, -100.0f, 0.30f, 1.0f);
    EXPECT_NEAR(s_big, 1.0f, 1e-6f, "out-of-range slip clamped to 1");
}

/* ---- bang-bang logic ---- */

static void test_bangbang_release_when_slip_high(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(20.0f, 50.0f, 100.0f);
    float cmd = abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.30f, 100.0f);
    EXPECT_NEAR(cmd, 0.0f, 1e-6f, "slip>UPPER -> release");
}

static void test_bangbang_reapply_when_slip_low(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(20.0f, 66.0f, 100.0f);
    float cmd = abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.05f, 100.0f);
    EXPECT_NEAR(cmd, 100.0f, 1e-6f, "slip<LOWER -> driver request");
}

static void test_bangbang_hysteresis_holds(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(20.0f, 60.0f, 100.0f);
    /* First push UPPER -> release. */
    abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.25f, 100.0f);
    /* Now slip in the dead zone -> should hold previous (= 0). */
    float cmd = abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.15f, 100.0f);
    EXPECT_NEAR(cmd, 0.0f, 1e-6f, "hysteresis holds previous=0 after release");
    /* Then drop below LOWER -> driver request. */
    cmd = abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.05f, 100.0f);
    EXPECT_NEAR(cmd, 100.0f, 1e-6f, "after release, slip<LOWER re-applies");
    /* In dead zone again -> hold (=100). */
    cmd = abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.15f, 100.0f);
    EXPECT_NEAR(cmd, 100.0f, 1e-6f, "hysteresis holds previous=100 after re-apply");
}

/* ---- fail-operational ---- */

static void test_degraded_state_passes_driver_through(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(20.0f, 60.0f, 100.0f);
    /* Even if slip is "ABS should release", in degraded the pedal wins. */
    float cmd = abs_controller_compute(&ctx, ECU_FAULT_DEGRADED, &s, 0.30f, 100.0f);
    EXPECT_NEAR(cmd, 100.0f, 1e-6f, "FAULT_DEGRADED -> pedal through");
}

static void test_latched_state_passes_driver_through(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(20.0f, 60.0f, 100.0f);
    float cmd = abs_controller_compute(&ctx, ECU_FAULT_LATCHED, &s, 0.30f, 80.0f);
    EXPECT_NEAR(cmd, 80.0f, 1e-6f, "FAULT_LATCHED -> pedal through");
}

static void test_low_speed_bypasses(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(0.5f, 0.0f, 100.0f);   /* below 5 km/h */
    float cmd = abs_controller_compute(&ctx, ECU_MONITOR, &s, 0.30f, 100.0f);
    EXPECT_NEAR(cmd, 100.0f, 1e-6f, "v<5km/h -> bypass to pedal");
}

static void test_standby_passes_driver_through(void) {
    abs_controller_ctx_t ctx; abs_controller_init(&ctx);
    sensors_data_t s = mk(20.0f, 60.0f, 0.0f);
    float cmd = abs_controller_compute(&ctx, ECU_STANDBY, &s, 0.30f, 100.0f);
    EXPECT_NEAR(cmd, 100.0f, 1e-6f, "STANDBY -> pedal through");
}

int main(void) {
    test_slip_free_wheel();
    test_slip_locked_wheel();
    test_slip_protected_at_zero_speed();
    test_slip_protected_clamps_to_unit_interval();
    test_bangbang_release_when_slip_high();
    test_bangbang_reapply_when_slip_low();
    test_bangbang_hysteresis_holds();
    test_degraded_state_passes_driver_through();
    test_latched_state_passes_driver_through();
    test_low_speed_bypasses();
    test_standby_passes_driver_through();

    if (failed) {
        fprintf(stderr, "\n%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    printf("\nall ABS controller tests passed.\n");
    return EXIT_SUCCESS;
}
