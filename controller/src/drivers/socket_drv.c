#define _POSIX_C_SOURCE 200809L

#include "socket_drv.h"
#include "crc.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

/* Receive buffer big enough for several worst-case frames. */
#define RX_BUFFER_SIZE  256u

struct socket_drv {
    int fd;
    uint8_t rx_buf[RX_BUFFER_SIZE];
    size_t rx_len;
    socket_drv_stats_t stats;
};

/* ---- helpers --------------------------------------------------------- */

static int set_tcp_nodelay(int fd) {
    int yes = 1;
    return setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
}

/* Send exactly `len` bytes — loops over partial writes, retries EINTR. */
static int send_all(int fd, const uint8_t *buf, size_t len) {
    size_t off = 0;
    while (off < len) {
        ssize_t n = send(fd, buf + off, len - off, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        off += (size_t)n;
    }
    return 0;
}

/* ---- public API ------------------------------------------------------ */

socket_drv_t *socket_drv_connect(const char *host, uint16_t port) {
    if (host == NULL) return NULL;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return NULL;

    if (set_tcp_nodelay(fd) != 0) {
        int e = errno; close(fd); errno = e; return NULL;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        int e = errno; close(fd); errno = e ? e : EINVAL; return NULL;
    }

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        int e = errno; close(fd); errno = e; return NULL;
    }

    socket_drv_t *drv = calloc(1, sizeof(*drv));
    if (drv == NULL) { close(fd); errno = ENOMEM; return NULL; }
    drv->fd = fd;
    return drv;
}

void socket_drv_close(socket_drv_t *drv) {
    if (drv == NULL) return;
    if (drv->fd >= 0) close(drv->fd);
    free(drv);
}

int socket_drv_send_actuator(socket_drv_t *drv,
                             const actuator_payload_t *payload,
                             uint32_t timestamp_ms_unused) {
    (void)timestamp_ms_unused;
    if (drv == NULL || payload == NULL) return -1;

    /* Envelope: magic | type | length | payload | crc */
    uint8_t buf[PROTO_OVERHEAD + sizeof(actuator_payload_t)];
    uint16_t magic = PROTO_MAGIC;
    uint16_t length = (uint16_t)sizeof(actuator_payload_t);

    buf[0] = (uint8_t)(magic & 0xFF);
    buf[1] = (uint8_t)((magic >> 8) & 0xFF);
    buf[2] = PROTO_TYPE_ACTUATOR;
    buf[3] = (uint8_t)(length & 0xFF);
    buf[4] = (uint8_t)((length >> 8) & 0xFF);
    memcpy(&buf[PROTO_HEADER_SIZE], payload, sizeof(*payload));

    uint16_t crc = crc16_ccitt(&buf[2], 1 + 2 + sizeof(*payload));
    size_t crc_off = PROTO_HEADER_SIZE + sizeof(*payload);
    buf[crc_off]     = (uint8_t)(crc & 0xFF);
    buf[crc_off + 1] = (uint8_t)((crc >> 8) & 0xFF);

    if (send_all(drv->fd, buf, sizeof(buf)) != 0) return -1;
    drv->stats.frames_sent_ok++;
    return 0;
}

/* Try to extract one valid sensor frame from the rx buffer. Returns:
 *   +1  ok, frame copied into out_payload
 *    0  buffer doesn't yet hold a full frame (caller must read more)
 *   -1  fatal (impossible here — only soft errors counted in stats)
 *
 * On bad frames (CRC fail, oversized length, wrong type), we drop bytes and
 * retry within the same call to amortise resync. */
static int try_extract_frame(socket_drv_t *drv, sensor_payload_t *out) {
    while (1) {
        /* Scan for the magic 0xAA55 inside the buffer. */
        size_t scan = 0;
        while (scan + 1 < drv->rx_len) {
            if (drv->rx_buf[scan] == (PROTO_MAGIC & 0xFF) &&
                drv->rx_buf[scan + 1] == ((PROTO_MAGIC >> 8) & 0xFF)) {
                break;
            }
            scan++;
        }
        if (scan + 1 >= drv->rx_len) {
            /* No magic found — drop everything except a possible dangling byte. */
            size_t keep = (drv->rx_len > 0) ? 1u : 0u;
            drv->stats.bytes_dropped += (uint32_t)(drv->rx_len - keep);
            if (keep) drv->rx_buf[0] = drv->rx_buf[drv->rx_len - 1];
            drv->rx_len = keep;
            return 0;
        }
        if (scan > 0) {
            drv->stats.bytes_dropped += (uint32_t)scan;
            memmove(drv->rx_buf, drv->rx_buf + scan, drv->rx_len - scan);
            drv->rx_len -= scan;
        }

        /* Need header to know the size. */
        if (drv->rx_len < PROTO_HEADER_SIZE + PROTO_CRC_SIZE) return 0;

        uint8_t  type   = drv->rx_buf[2];
        uint16_t length = (uint16_t)drv->rx_buf[3] |
                          ((uint16_t)drv->rx_buf[4] << 8);

        if (length > PROTO_MAX_PAYLOAD) {
            drv->stats.errors_size++;
            /* Slide past the magic and rescan. */
            memmove(drv->rx_buf, drv->rx_buf + 2, drv->rx_len - 2);
            drv->rx_len -= 2;
            continue;
        }
        if (type != PROTO_TYPE_SENSOR) {
            drv->stats.errors_type++;
            memmove(drv->rx_buf, drv->rx_buf + 2, drv->rx_len - 2);
            drv->rx_len -= 2;
            continue;
        }

        size_t total = (size_t)PROTO_HEADER_SIZE + length + PROTO_CRC_SIZE;
        if (drv->rx_len < total) return 0;

        uint16_t crc_recv = (uint16_t)drv->rx_buf[PROTO_HEADER_SIZE + length] |
                            ((uint16_t)drv->rx_buf[PROTO_HEADER_SIZE + length + 1] << 8);
        uint16_t crc_calc = crc16_ccitt(&drv->rx_buf[2], 1 + 2 + length);

        if (crc_recv != crc_calc) {
            drv->stats.errors_crc++;
            memmove(drv->rx_buf, drv->rx_buf + 2, drv->rx_len - 2);
            drv->rx_len -= 2;
            continue;
        }

        /* Sensor frames must match the agreed payload size exactly. */
        if (length != sizeof(sensor_payload_t)) {
            drv->stats.errors_size++;
            memmove(drv->rx_buf, drv->rx_buf + total, drv->rx_len - total);
            drv->rx_len -= total;
            continue;
        }

        memcpy(out, &drv->rx_buf[PROTO_HEADER_SIZE], sizeof(*out));
        memmove(drv->rx_buf, drv->rx_buf + total, drv->rx_len - total);
        drv->rx_len -= total;
        drv->stats.frames_received_ok++;
        return 1;
    }
}

int socket_drv_recv_sensor(socket_drv_t *drv,
                           sensor_payload_t *out_payload,
                           int timeout_ms) {
    if (drv == NULL || out_payload == NULL) return -1;

    /* First, try to extract from what we already buffered. */
    int rc = try_extract_frame(drv, out_payload);
    if (rc == 1) return 1;

    while (1) {
        struct pollfd pfd = { .fd = drv->fd, .events = POLLIN };
        int pr = poll(&pfd, 1, timeout_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (pr == 0) return 0;        /* timeout */

        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
        if (!(pfd.revents & POLLIN)) return -1;

        if (drv->rx_len >= RX_BUFFER_SIZE) {
            /* Should never happen given the small frame size, but be safe. */
            drv->stats.bytes_dropped += (uint32_t)drv->rx_len;
            drv->rx_len = 0;
        }
        ssize_t n = recv(drv->fd, drv->rx_buf + drv->rx_len,
                         RX_BUFFER_SIZE - drv->rx_len, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;        /* peer closed */
        drv->rx_len += (size_t)n;

        rc = try_extract_frame(drv, out_payload);
        if (rc == 1) return 1;
        /* otherwise loop and read more */
    }
}

void socket_drv_stats(const socket_drv_t *drv, socket_drv_stats_t *out) {
    if (drv == NULL || out == NULL) return;
    *out = drv->stats;
}
