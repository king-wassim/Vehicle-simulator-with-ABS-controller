#ifndef ABS_HAL_ACTUATORS_H
#define ABS_HAL_ACTUATORS_H

/**
 * Actuator HAL — the only interface the app code uses to command the brake.
 *
 * Symmetric to hal_sensors: today it ships an actuator_payload_t over TCP;
 * on the real ECU it will drive a hydraulic valve via PWM. The signature
 * does not change.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Application-facing brake command + ECU telemetry returned to the plant. */
typedef struct {
    uint32_t timestamp_ms;  /* local clock at decision time */
    float    brake_command; /* bar */
    uint16_t ecu_status;    /* see ecu_status_t in protocol.h */
    uint16_t dtc_code;      /* DTC bitfield */
} actuator_data_t;

/**
 * `sensors_drv` MUST be the same socket previously opened by
 * hal_sensors_init() — the SIL link is full-duplex on a single TCP
 * connection. We share the file descriptor instead of opening a second one.
 *
 * Returns 0 on success, -1 on error.
 */
int  hal_actuators_init(void);

void hal_actuators_shutdown(void);

/** Send one actuator frame. Returns 0 on success, -1 on transport error. */
int hal_actuators_write(const actuator_data_t *cmd);

typedef struct {
    uint32_t frames_sent;
    uint32_t send_errors;
} hal_actuators_stats_t;

void hal_actuators_stats(hal_actuators_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ABS_HAL_ACTUATORS_H */
