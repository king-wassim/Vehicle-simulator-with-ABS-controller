#ifndef ABS_HAL_INTERNAL_H
#define ABS_HAL_INTERNAL_H

/**
 * Internal HAL header — NOT for app code.
 *
 * Exposes shared handles between HAL modules (hal_sensors, hal_actuators)
 * which both ride on the same physical link in this SIL setup. On a real
 * ECU, sensors would come from CAN/ADC and the actuator from PWM/SPI — two
 * independent drivers. We keep the same surface for the app and just
 * special-case the SIL coupling here.
 */

#include "socket_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* hal_sensors_init() registers the driver here; hal_actuators reads it. */
void           hal_link_set(socket_drv_t *drv);
socket_drv_t  *hal_link_get(void);

#ifdef __cplusplus
}
#endif

#endif /* ABS_HAL_INTERNAL_H */
