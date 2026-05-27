#include "hal_actuators.h"
#include "hal_internal.h"

#include <stddef.h>
#include <string.h>

#include "protocol.h"
#include "socket_drv.h"

static struct {
    bool         initialised;
    uint32_t     frames_sent;
    uint32_t     send_errors;
} g_hal;

int hal_actuators_init(void) {
    if (hal_link_get() == NULL) return -1;       /* sensors must init first */
    g_hal.initialised = true;
    g_hal.frames_sent = 0;
    g_hal.send_errors = 0;
    return 0;
}

void hal_actuators_shutdown(void) {
    g_hal.initialised = false;
}

int hal_actuators_write(const actuator_data_t *cmd) {
    if (cmd == NULL || !g_hal.initialised) return -1;
    socket_drv_t *drv = hal_link_get();
    if (drv == NULL) return -1;

    actuator_payload_t pl;
    pl.timestamp_ms  = cmd->timestamp_ms;
    pl.brake_command = cmd->brake_command;
    pl.ecu_status    = cmd->ecu_status;
    pl.dtc_code      = cmd->dtc_code;

    if (socket_drv_send_actuator(drv, &pl, cmd->timestamp_ms) != 0) {
        g_hal.send_errors++;
        return -1;
    }
    g_hal.frames_sent++;
    return 0;
}

void hal_actuators_stats(hal_actuators_stats_t *out) {
    if (out == NULL) return;
    out->frames_sent = g_hal.frames_sent;
    out->send_errors = g_hal.send_errors;
}
