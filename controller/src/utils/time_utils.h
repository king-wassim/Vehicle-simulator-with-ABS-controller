#ifndef ABS_TIME_UTILS_H
#define ABS_TIME_UTILS_H

#define _POSIX_C_SOURCE 200809L

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Add `ns` nanoseconds to a timespec, normalising tv_nsec into [0, 1e9). */
static inline void timespec_add_ns(struct timespec *ts, long ns) {
    long total = ts->tv_nsec + ns;
    ts->tv_sec  += total / 1000000000L;
    ts->tv_nsec  = total % 1000000000L;
    if (ts->tv_nsec < 0) {
        ts->tv_nsec += 1000000000L;
        ts->tv_sec  -= 1;
    }
}

/** Difference b - a in microseconds. Can be negative. */
static inline long timespec_diff_us(const struct timespec *a,
                                    const struct timespec *b) {
    return (b->tv_sec - a->tv_sec) * 1000000L
         + (b->tv_nsec - a->tv_nsec) / 1000L;
}

/** Monotonic-clock milliseconds since some unspecified epoch, 32-bit wrap. */
static inline uint32_t now_ms_mono(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

#ifdef __cplusplus
}
#endif

#endif /* ABS_TIME_UTILS_H */
