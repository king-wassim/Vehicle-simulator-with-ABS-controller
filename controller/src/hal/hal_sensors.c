#define _POSIX_C_SOURCE 200809L

#include "hal_sensors.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "hal_internal.h"
#include "protocol.h"
#include "socket_drv.h"

/* Single-instance HAL — fine for SIL with one ECU per process. */
static struct {
    socket_drv_t *drv;
    bool          alive;
    uint32_t      timeouts;
} g_hal;

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

int hal_sensors_init(const char *host, uint16_t port) {
    if (g_hal.drv != NULL) return 0;     /* idempotent */
    g_hal.drv = socket_drv_connect(host, port);
    if (g_hal.drv == NULL) return -1;
    hal_link_set(g_hal.drv);
    g_hal.alive = true;
    g_hal.timeouts = 0;
    return 0;
}

void hal_sensors_shutdown(void) {
    if (g_hal.drv != NULL) {
        socket_drv_close(g_hal.drv);
        g_hal.drv = NULL;
        hal_link_set(NULL);
    }
    g_hal.alive = false;
}

int hal_sensors_read(sensors_data_t *out, int timeout_ms) {
    if (out == NULL || g_hal.drv == NULL) return -1;

    sensor_payload_t raw;
    int rc = socket_drv_recv_sensor(g_hal.drv, &raw, timeout_ms);
    if (rc == 0) {
        g_hal.timeouts++;
        out->valid = false;
        return -1;
    }
    if (rc < 0) {
        g_hal.alive = false;
        out->valid = false;
        return -1;
    }

    out->timestamp_ms   = raw.timestamp_ms;
    out->rx_time_ms     = now_ms();
    out->v_vehicle      = raw.v_vehicle;
    out->omega_wheel    = raw.omega_wheel;
    out->brake_pressure = raw.brake_pressure;
    out->fault_flags    = raw.fault_flags;
    out->valid          = true;
    return 0;
}

bool hal_sensors_link_alive(void) {
    return g_hal.alive;
}

void hal_sensors_stats(hal_sensors_stats_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (g_hal.drv != NULL) {
        socket_drv_stats_t s;
        socket_drv_stats(g_hal.drv, &s);
        out->frames_ok     = s.frames_received_ok;
        out->errors_crc    = s.errors_crc;
        out->errors_size   = s.errors_size;
        out->errors_type   = s.errors_type;
        out->bytes_dropped = s.bytes_dropped;
    }
    out->timeouts = g_hal.timeouts;
}
