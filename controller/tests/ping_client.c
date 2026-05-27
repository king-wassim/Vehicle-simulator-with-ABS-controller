/**
 * Week-1 milestone client.
 *
 * Connects to 127.0.0.1:9000, then for each sensor frame received echoes
 * back an actuator frame whose `brake_command` mirrors `v_vehicle` so the
 * server can pin-test the round-trip without any control logic. Exits after
 * N frames or when the peer closes.
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O2 \
 *       controller/tests/ping_client.c \
 *       controller/src/drivers/crc.c controller/src/drivers/socket_drv.c \
 *       -Icontroller/src/drivers \
 *       -o controller/build/ping_client
 */

#define _POSIX_C_SOURCE 200809L

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "socket_drv.h"
#include "protocol.h"

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

int main(int argc, char **argv) {
    const char *host = (argc > 1) ? argv[1] : "127.0.0.1";
    uint16_t port = (argc > 2) ? (uint16_t)atoi(argv[2]) : 9000u;
    uint32_t max_frames = (argc > 3) ? (uint32_t)atol(argv[3]) : 0u;  /* 0 = forever */

    signal(SIGINT, on_sigint);
    signal(SIGPIPE, SIG_IGN);

    socket_drv_t *drv = socket_drv_connect(host, port);
    if (drv == NULL) {
        perror("connect");
        return EXIT_FAILURE;
    }
    fprintf(stderr, "ping_client: connected to %s:%u\n", host, (unsigned)port);

    uint32_t handled = 0;
    while (!g_stop) {
        sensor_payload_t s;
        int rc = socket_drv_recv_sensor(drv, &s, 1000);
        if (rc == 0) {
            fprintf(stderr, "ping_client: idle (no frame in 1s)\n");
            continue;
        }
        if (rc < 0) {
            fprintf(stderr, "ping_client: peer closed or I/O error\n");
            break;
        }

        actuator_payload_t a;
        a.timestamp_ms  = now_ms();
        a.brake_command = s.v_vehicle;          /* echo for the milestone test */
        a.ecu_status    = ECU_MONITOR;
        a.dtc_code      = DTC_NONE;
        if (socket_drv_send_actuator(drv, &a, a.timestamp_ms) != 0) {
            fprintf(stderr, "ping_client: send failed\n");
            break;
        }

        handled++;
        if (max_frames && handled >= max_frames) {
            fprintf(stderr, "ping_client: handled %u frames, exiting\n", handled);
            break;
        }
    }

    socket_drv_stats_t st;
    socket_drv_stats(drv, &st);
    printf("ping_client: rx_ok=%u tx_ok=%u crc_err=%u size_err=%u "
           "type_err=%u dropped_bytes=%u\n",
           st.frames_received_ok, st.frames_sent_ok,
           st.errors_crc, st.errors_size, st.errors_type, st.bytes_dropped);

    socket_drv_close(drv);
    return (st.errors_crc || st.errors_size || st.errors_type) ? 1 : 0;
}
