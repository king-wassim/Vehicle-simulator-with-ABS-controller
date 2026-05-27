#ifndef ABS_SOCKET_DRV_H
#define ABS_SOCKET_DRV_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * "CAN-like" socket driver — encapsulates everything POSIX about the link.
 * Above this API, the rest of the firmware (HAL, app) sees no `socket.h`.
 *
 * One process role only: TCP client. The plant is the server.
 */

typedef struct socket_drv socket_drv_t;

/* Open and connect to host:port. Sets TCP_NODELAY and SO_KEEPALIVE.
 * Returns NULL on failure (errno set). */
socket_drv_t *socket_drv_connect(const char *host, uint16_t port);

void socket_drv_close(socket_drv_t *drv);

/* Send one fully-framed (envelope+payload+crc) actuator frame.
 * Returns 0 on success, -1 on error. Partial writes are looped internally. */
int socket_drv_send_actuator(socket_drv_t *drv,
                             const actuator_payload_t *payload,
                             uint32_t timestamp_ms_unused);

/* Block until one valid sensor frame is decoded, or until `timeout_ms`
 * elapses. Returns:
 *    +1  on a valid frame (out_payload populated)
 *     0  on timeout
 *    -1  on a hard I/O error (caller should close and reconnect)
 *
 * Invalid frames (bad CRC, bad size, garbage bytes) are silently dropped
 * and counted; query stats via socket_drv_stats(). */
int socket_drv_recv_sensor(socket_drv_t *drv,
                           sensor_payload_t *out_payload,
                           int timeout_ms);

typedef struct {
    uint32_t frames_received_ok;
    uint32_t frames_sent_ok;
    uint32_t errors_crc;
    uint32_t errors_size;
    uint32_t errors_type;
    uint32_t bytes_dropped;
} socket_drv_stats_t;

void socket_drv_stats(const socket_drv_t *drv, socket_drv_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ABS_SOCKET_DRV_H */
