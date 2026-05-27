#ifndef ABS_CONTROLLER_H
#define ABS_CONTROLLER_H

/**
 * ABS controller — bang-bang with hysteresis.
 *
 * This is the C twin of `plant/oracle_abs.py`. Same thresholds, same
 * algorithm. Keeping them aligned is what lets the SIL stopping distance
 * match the offline reference.
 *
 * Control law (per cycle):
 *      if slip > UPPER:   command = 0          (release)
 *      elif slip < LOWER: command = driver     (re-apply)
 *      else:              command = previous   (hysteresis hold)
 *
 * Below ABS_LOW_SPEED_BYPASS_MPS the loop hands over to the driver pedal
 * because slip is ill-defined at near-zero speed.
 *
 * In FAULT_DEGRADED / FAULT_LATCHED the loop also hands over to the driver
 * (fail-operational: the driver must still be able to brake — without ABS
 * modulation — even when our sensors are flaky). See ARCHITECTURE.md §6.
 */

#include <stdbool.h>
#include <stdint.h>

#include "abs_state.h"
#include "hal_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Caller-owned per-instance state — keeps `previous_command` for hysteresis. */
typedef struct {
    float previous_command;   /* bar */
} abs_controller_ctx_t;

void abs_controller_init(abs_controller_ctx_t *ctx);

/** Pure slip ratio (no protection — caller decides what to do at v→0). */
float abs_controller_slip(float v_vehicle, float omega_wheel, float wheel_radius);

/**
 * Slip with the v→0 guard that pairs with the plant model. Returns 0 when
 * the vehicle speed is below `v_min`. Used by the super-loop so we don't
 * have to repeat the check at every call site.
 */
float abs_controller_slip_protected(float v_vehicle, float omega_wheel,
                                    float wheel_radius, float v_min);

/**
 * Compute the next brake command.
 *
 *   @param ctx              controller hysteresis state (caller-owned)
 *   @param state            current ECU state (selects bypass behaviour)
 *   @param sensors          last sensor sample
 *   @param slip             slip ratio (already protected against v→0)
 *   @param driver_request   pedal position translated to bar (0..150)
 */
float abs_controller_compute(abs_controller_ctx_t *ctx,
                             ecu_status_t          state,
                             const sensors_data_t *sensors,
                             float                 slip,
                             float                 driver_request);

#ifdef __cplusplus
}
#endif

#endif /* ABS_CONTROLLER_H */
