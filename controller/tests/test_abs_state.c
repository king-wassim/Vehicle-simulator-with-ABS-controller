/**
 * State machine unit tests — covers each documented transition plus the
 * fault-handling shortcuts.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
 *       controller/tests/test_abs_state.c controller/src/app/abs_state.c \
 *       -Icontroller/src/app -Icontroller/src/hal -Icontroller/src/drivers \
 *       -o controller/build/test_abs_state
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "abs_state.h"

static int failed = 0;

#define EXPECT_STATE(ctx, want, label)                                       \
    do {                                                                     \
        if ((ctx).current != (want)) {                                       \
            fprintf(stderr, "FAIL %s: expected %s, got %s\n",                \
                    (label), abs_state_name((want)),                         \
                    abs_state_name((ctx).current));                          \
            failed++;                                                        \
        } else {                                                             \
            printf("  ok  %s -> %s\n", (label), abs_state_name((want)));     \
        }                                                                    \
    } while (0)

static sensors_data_t make_sensors(float v, float omega, float pbar) {
    sensors_data_t s = {0};
    s.valid          = true;
    s.v_vehicle      = v;
    s.omega_wheel    = omega;
    s.brake_pressure = pbar;
    return s;
}

/* ---- Tests ----------------------------------------------------------- */

static void test_init_to_monitor_when_moving(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 0.0f);
    abs_state_update(&ctx, &s, 0.0f, DTC_NONE);
    EXPECT_STATE(ctx, ECU_MONITOR, "INIT + moving -> MONITOR");
}

static void test_init_to_standby_when_stopped(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(0.0f, 0.0f, 0.0f);
    abs_state_update(&ctx, &s, 0.0f, DTC_NONE);
    EXPECT_STATE(ctx, ECU_STANDBY, "INIT + stopped -> STANDBY");
}

static void test_standby_to_monitor_when_braking(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(0.0f, 0.0f, 0.0f);
    abs_state_update(&ctx, &s, 0.0f, DTC_NONE);              /* -> STANDBY */
    s = make_sensors(15.0f, 50.0f, 60.0f);                   /* moving + brake */
    abs_state_update(&ctx, &s, 0.05f, DTC_NONE);
    EXPECT_STATE(ctx, ECU_MONITOR, "STANDBY + moving + brake -> MONITOR");
}

static void test_monitor_to_active_when_slip_high(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.05f, DTC_NONE);             /* INIT -> MONITOR */
    abs_state_update(&ctx, &s, 0.25f, DTC_NONE);             /* slip overshoot */
    EXPECT_STATE(ctx, ECU_ACTIVE, "MONITOR + slip>0.20 -> ACTIVE");
}

static void test_active_to_monitor_when_slip_low(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.05f, DTC_NONE);
    abs_state_update(&ctx, &s, 0.30f, DTC_NONE);             /* MONITOR -> ACTIVE */
    abs_state_update(&ctx, &s, 0.08f, DTC_NONE);             /* slip<0.10 */
    EXPECT_STATE(ctx, ECU_MONITOR, "ACTIVE + slip<0.10 -> MONITOR");
}

static void test_monitor_to_standby_when_stops(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.05f, DTC_NONE);             /* -> MONITOR */
    s = make_sensors(0.5f, 0.0f, 50.0f);                     /* stopped */
    abs_state_update(&ctx, &s, 0.0f, DTC_NONE);
    EXPECT_STATE(ctx, ECU_STANDBY, "MONITOR + v<5km/h -> STANDBY");
}

static void test_dtc_forces_degraded(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.30f, DTC_NONE);             /* -> MONITOR */
    abs_state_update(&ctx, &s, 0.30f, DTC_SENSOR_NOISE);     /* DTC raised */
    EXPECT_STATE(ctx, ECU_FAULT_DEGRADED, "any + DTC -> FAULT_DEGRADED");
}

static void test_persistent_dtc_latches(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.30f, DTC_NONE);             /* MONITOR */
    /* Fire enough faults to latch. */
    for (unsigned i = 0; i < ABS_FAULT_LATCH_THRESHOLD; ++i) {
        abs_state_update(&ctx, &s, 0.30f, DTC_PLAUSIBILITY);
    }
    EXPECT_STATE(ctx, ECU_FAULT_LATCHED, "N consecutive DTCs -> FAULT_LATCHED");
    /* A clean sample must NOT clear the latch. */
    abs_state_update(&ctx, &s, 0.30f, DTC_NONE);
    EXPECT_STATE(ctx, ECU_FAULT_LATCHED, "FAULT_LATCHED is sticky");
}

static void test_degraded_recovers_to_monitor(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.30f, DTC_NONE);
    abs_state_update(&ctx, &s, 0.30f, DTC_SENSOR_NOISE);     /* degraded */
    abs_state_update(&ctx, &s, 0.30f, DTC_NONE);             /* recovered */
    EXPECT_STATE(ctx, ECU_MONITOR, "FAULT_DEGRADED + clean -> MONITOR");
}

static void test_invalid_sensors_are_a_fault(void) {
    abs_state_ctx_t ctx; abs_state_init(&ctx);
    sensors_data_t s = make_sensors(20.0f, 66.0f, 80.0f);
    abs_state_update(&ctx, &s, 0.05f, DTC_NONE);             /* MONITOR */
    s.valid = false;
    abs_state_update(&ctx, &s, 0.0f, DTC_NONE);
    EXPECT_STATE(ctx, ECU_FAULT_DEGRADED, "invalid sensors -> FAULT_DEGRADED");
}

int main(void) {
    test_init_to_monitor_when_moving();
    test_init_to_standby_when_stopped();
    test_standby_to_monitor_when_braking();
    test_monitor_to_active_when_slip_high();
    test_active_to_monitor_when_slip_low();
    test_monitor_to_standby_when_stops();
    test_dtc_forces_degraded();
    test_persistent_dtc_latches();
    test_degraded_recovers_to_monitor();
    test_invalid_sensors_are_a_fault();

    if (failed) {
        fprintf(stderr, "\n%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    printf("\nall state machine tests passed.\n");
    return EXIT_SUCCESS;
}
