#define _POSIX_C_SOURCE 200809L

/**
 * ABS ECU main — super-loop, 100 Hz, hard-wired to the SIL plant.
 *
 *      ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
 *      │ hal_sensors  │──►│ diagnostic   │──►│ abs_state    │
 *      └──────────────┘    └──────────────┘    └──────┬───────┘
 *                                                     │
 *                                              ┌──────▼───────┐
 *                                              │ abs_controller│
 *                                              └──────┬───────┘
 *                                                     ▼
 *                                              ┌──────────────┐
 *                                              │ hal_actuators│
 *                                              └──────────────┘
 *
 * Timing: clock_nanosleep(TIMER_ABSTIME) on a 10 ms grid — no drift even if
 * a cycle runs long. Each loop logs its wakeup jitter (real - scheduled)
 * for the W2 milestone goal of σ < 1 ms over 60 s.
 *
 * Usage:
 *   abs_ecu [host] [port] [duration_s] [driver_request_bar]
 *      host                = 127.0.0.1
 *      port                = 9000
 *      duration_s          = 0   (run forever)
 *      driver_request_bar  = 100 (panic-brake default; 0 = no braking)
 */

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "abs_controller.h"
#include "abs_state.h"
#include "hal_actuators.h"
#include "hal_sensors.h"
#include "protocol.h"
#include "time_utils.h"

/* ---- configuration ---------------------------------------------------- */

#define LOOP_PERIOD_NS        (10L * 1000L * 1000L)   /* 10 ms = 100 Hz */
#define SENSOR_TIMEOUT_MS     (5)                     /* must be < period */
#define WATCHDOG_TIMEOUT_MS   (50)                    /* arch §6 */
#define WHEEL_RADIUS_M        (0.30f)
#define SLIP_V_MIN_MPS        (1.0f)
#define LOG_EVERY_N_CYCLES    (50u)                   /* 2 Hz human log */

/* ---- globals ---------------------------------------------------------- */

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* Jitter accumulator — Welford for variance, plus min/max. */
typedef struct {
    uint64_t n;
    double   mean_us;
    double   m2_us;          /* sum of squared diffs from mean */
    long     min_us;
    long     max_us;
} jitter_stats_t;

static void jitter_init(jitter_stats_t *j) {
    j->n = 0; j->mean_us = 0.0; j->m2_us = 0.0;
    j->min_us = LONG_MAX; j->max_us = LONG_MIN;
}

static void jitter_add(jitter_stats_t *j, long us) {
    j->n++;
    double delta  = (double)us - j->mean_us;
    j->mean_us   += delta / (double)j->n;
    double delta2 = (double)us - j->mean_us;
    j->m2_us     += delta * delta2;
    if (us < j->min_us) j->min_us = us;
    if (us > j->max_us) j->max_us = us;
}

static double jitter_stddev_us(const jitter_stats_t *j) {
    if (j->n < 2) return 0.0;
    return sqrt(j->m2_us / (double)(j->n - 1));
}

/* ---- minimal in-loop diagnostic (full FMEA module lands in week 3) ---- */

static uint16_t diagnostic_quick(const sensors_data_t *s,
                                 uint32_t cycles_since_last_valid) {
    uint16_t dtc = DTC_NONE;
    /* F03: comm watchdog — too long without a fresh frame. */
    if (cycles_since_last_valid * (uint32_t)(LOOP_PERIOD_NS / 1000000L)
        > (uint32_t)WATCHDOG_TIMEOUT_MS) {
        dtc |= DTC_COMM_TIMEOUT;
    }
    if (s == NULL || !s->valid) return dtc;
    /* F05: gross range check — same envelope as in abs_state. */
    if (s->omega_wheel < -0.1f || s->omega_wheel > 500.0f) dtc |= DTC_SENSOR_RANGE;
    if (s->v_vehicle   < -1.0f || s->v_vehicle  > 120.0f)  dtc |= DTC_SENSOR_RANGE;
    return dtc;
}

/* ---- main loop -------------------------------------------------------- */

int main(int argc, char **argv) {
    const char *host           = (argc > 1) ? argv[1]              : "127.0.0.1";
    uint16_t    port           = (argc > 2) ? (uint16_t)atoi(argv[2]) : 9000u;
    double      duration_s     = (argc > 3) ? atof(argv[3])          : 0.0;
    float       driver_request = (argc > 4) ? (float)atof(argv[4])   : 100.0f;

    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);
    signal(SIGPIPE, SIG_IGN);

    if (hal_sensors_init(host, port) != 0) {
        fprintf(stderr, "abs_ecu: hal_sensors_init failed: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    if (hal_actuators_init() != 0) {
        fprintf(stderr, "abs_ecu: hal_actuators_init failed\n");
        hal_sensors_shutdown();
        return EXIT_FAILURE;
    }

    fprintf(stderr, "abs_ecu: connected to %s:%u, driver pedal=%.0f bar, "
                    "duration=%.1fs\n",
            host, (unsigned)port, (double)driver_request, duration_s);

    abs_state_ctx_t      st;     abs_state_init(&st);
    abs_controller_ctx_t cc;     abs_controller_init(&cc);
    jitter_stats_t       jit;    jitter_init(&jit);

    sensors_data_t  last_good = {0};
    sensors_data_t  cur       = {0};
    uint32_t        cycles_since_valid = 0;
    uint64_t        cycle = 0;
    uint32_t        max_cycles = (duration_s > 0.0)
        ? (uint32_t)(duration_s * 1.0e9 / (double)LOOP_PERIOD_NS) : 0;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (!g_stop) {
        /* --- 1. wait until the next 10 ms tick --- */
        timespec_add_ns(&next, LOOP_PERIOD_NS);
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (rc != 0 && rc != EINTR) {
            fprintf(stderr, "abs_ecu: clock_nanosleep: %s\n", strerror(rc));
            break;
        }

        /* Measure jitter: real wakeup vs scheduled. */
        struct timespec real_wake;
        clock_gettime(CLOCK_MONOTONIC, &real_wake);
        jitter_add(&jit, timespec_diff_us(&next, &real_wake));

        /* --- 2. read sensor (non-blocking-ish, 5 ms budget) --- */
        int r = hal_sensors_read(&cur, SENSOR_TIMEOUT_MS);
        if (r == 0) {
            last_good = cur;
            cycles_since_valid = 0;
        } else {
            cycles_since_valid++;
            cur.valid = false;
        }

        /* --- 3. quick diagnostic (full FMEA in week 3) --- */
        const sensors_data_t *for_state =
            (cur.valid) ? &cur :
            (cycles_since_valid * (uint32_t)(LOOP_PERIOD_NS / 1000000L)
                 <= (uint32_t)WATCHDOG_TIMEOUT_MS) ? &last_good : &cur;
        uint16_t dtc = diagnostic_quick(for_state, cycles_since_valid);

        /* --- 4. slip ratio (protected against v→0) --- */
        float slip = abs_controller_slip_protected(
            for_state->v_vehicle, for_state->omega_wheel,
            WHEEL_RADIUS_M, SLIP_V_MIN_MPS);

        /* --- 5. state machine + control --- */
        ecu_status_t new_state = abs_state_update(&st, for_state, slip, dtc);
        float cmd = abs_controller_compute(&cc, new_state, for_state,
                                            slip, driver_request);

        /* --- 6. emit actuator --- */
        actuator_data_t out;
        out.timestamp_ms  = now_ms_mono();
        out.brake_command = cmd;
        out.ecu_status    = (uint16_t)new_state;
        out.dtc_code      = dtc;
        if (hal_actuators_write(&out) != 0) {
            fprintf(stderr, "abs_ecu: hal_actuators_write failed at cycle %llu\n",
                    (unsigned long long)cycle);
            break;
        }

        /* --- 7. human log every 0.5 s --- */
        if ((cycle % LOG_EVERY_N_CYCLES) == 0) {
            fprintf(stderr,
                    "[t=%6.2fs] state=%-15s slip=%5.3f v=%5.2f m/s "
                    "omega=%6.2f rad/s brake=%5.1f bar dtc=0x%04X "
                    "jitter mean=%6.1f us σ=%5.1f us\n",
                    (double)cycle * (LOOP_PERIOD_NS / 1.0e9),
                    abs_state_name(new_state), (double)slip,
                    (double)for_state->v_vehicle, (double)for_state->omega_wheel,
                    (double)cmd, (unsigned)dtc,
                    jit.mean_us, jitter_stddev_us(&jit));
        }

        cycle++;
        if (max_cycles && cycle >= max_cycles) break;
    }

    /* ---- summary ---- */
    hal_sensors_stats_t   ss; hal_sensors_stats(&ss);
    hal_actuators_stats_t as; hal_actuators_stats(&as);
    printf("\n=== abs_ecu summary ===\n");
    printf("cycles run         : %llu\n", (unsigned long long)cycle);
    printf("sensors  rx_ok=%u crc_err=%u size_err=%u dropped=%u timeouts=%u\n",
           ss.frames_ok, ss.errors_crc, ss.errors_size,
           ss.bytes_dropped, ss.timeouts);
    printf("actuators tx_ok=%u send_err=%u\n", as.frames_sent, as.send_errors);
    printf("jitter (us)        : mean=%.2f stddev=%.2f min=%ld max=%ld over n=%llu\n",
           jit.mean_us, jitter_stddev_us(&jit),
           jit.min_us, jit.max_us, (unsigned long long)jit.n);
    printf("final state        : %s\n", abs_state_name(st.current));

    hal_actuators_shutdown();
    hal_sensors_shutdown();
    return EXIT_SUCCESS;
}
