#ifndef ABS_HAL_SENSORS_H
#define ABS_HAL_SENSORS_H

/**
 * Sensor HAL — the only interface the application code uses to read wheel
 * speed, vehicle speed, brake pressure feedback, and injected-fault flags.
 *
 * Why this layer exists:
 *   - On the SIL bench (today), `sensors_data_t` comes from a TCP socket.
 *   - On the real ECU (tomorrow), the same `sensors_data_t` will come from
 *     ADCs / CAN frames. The app code does not care which.
 *
 * This is exactly the AUTOSAR pattern: ECU-Abstraction-Layer above MCAL,
 * Application Software above the RTE. Here MCAL = socket_drv, RTE = the
 * function calls below.
 *
 * Threading model: single-threaded super-loop. None of these calls are
 * re-entrant.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Application-facing snapshot of one sensor sample. */
typedef struct {
    uint32_t timestamp_ms;   /* sender (plant) clock */
    uint32_t rx_time_ms;     /* local clock when this sample arrived */
    float    v_vehicle;      /* m/s   */
    float    omega_wheel;    /* rad/s */
    float    brake_pressure; /* bar — feedback from hydraulic line */
    uint16_t fault_flags;    /* informative bitfield from the plant injector */
    bool     valid;          /* false on timeout / I/O error */
} sensors_data_t;

/** Open and connect to the plant. Returns 0 on success, -1 on error (errno set). */
int  hal_sensors_init(const char *host, uint16_t port);

/** Tear down. Safe to call multiple times. */
void hal_sensors_shutdown(void);

/**
 * Pull the next sensor frame from the link.
 *
 * Blocks up to `timeout_ms` milliseconds. On timeout, returns a sample with
 * `valid = false` and leaves the other fields untouched (so callers can keep
 * the previous value if they choose). On hard I/O error, also returns
 * `valid = false`; the caller should ask `hal_sensors_link_alive()` to tell
 * the two cases apart.
 *
 * Returns 0 if a fresh valid sample was produced, -1 otherwise.
 */
int hal_sensors_read(sensors_data_t *out, int timeout_ms);

/** Is the underlying transport still up (i.e. peer connected, no fatal err)? */
bool hal_sensors_link_alive(void);

/** Telemetry — for the diagnostic module / DTCs. */
typedef struct {
    uint32_t frames_ok;
    uint32_t errors_crc;
    uint32_t errors_size;
    uint32_t errors_type;
    uint32_t bytes_dropped;
    uint32_t timeouts;
} hal_sensors_stats_t;

void hal_sensors_stats(hal_sensors_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ABS_HAL_SENSORS_H */
