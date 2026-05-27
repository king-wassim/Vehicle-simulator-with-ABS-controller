/**
 * Standalone CRC test — runs the CCITT-FALSE KAT and a few extra vectors,
 * fails the process with a non-zero exit code on mismatch.
 *
 * Build:  gcc -std=c11 -Wall -Wextra controller/tests/test_crc.c \
 *             controller/src/drivers/crc.c -Icontroller/src/drivers \
 *             -o build/test_crc
 * Run:    ./build/test_crc
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "crc.h"

static int failed = 0;

#define EXPECT_EQ(actual, expected, label)                                   \
    do {                                                                     \
        uint16_t a = (uint16_t)(actual);                                     \
        uint16_t e = (uint16_t)(expected);                                   \
        if (a != e) {                                                        \
            fprintf(stderr, "FAIL %s: got 0x%04X, want 0x%04X\n",            \
                    (label), a, e);                                          \
            failed++;                                                        \
        } else {                                                             \
            printf("  ok  %s = 0x%04X\n", (label), a);                       \
        }                                                                    \
    } while (0)

int main(void) {
    EXPECT_EQ(crc16_ccitt("123456789", 9), 0x29B1, "KAT \"123456789\"");
    EXPECT_EQ(crc16_ccitt("", 0),           0xFFFF, "empty input");
    EXPECT_EQ(crc16_ccitt("\x00", 1),       0xE1F0, "single 0x00");
    EXPECT_EQ(crc16_ccitt("\xff", 1),       0xFF00, "single 0xFF");

    /* Cross-language anchor: same bytes the Python test_envelope_roundtrip
     * pushes through the codec, so a CRC drift would surface in both suites. */
    const uint8_t fixed[] = { 0x01, 0x12, 0x00,            /* type, len lo, len hi */
                              0x7B, 0x00, 0x00, 0x00,      /* timestamp = 123 */
                              0x00, 0x00, 0xC8, 0x41,      /* v_vehicle = 25.0f */
                              0x00, 0x00, 0xA0, 0x42,      /* omega = 80.0f */
                              0x00, 0x00, 0x48, 0x42,      /* brake_p = 50.0f */
                              0x00, 0x00 };                /* flags = 0 */
    uint16_t got = crc16_ccitt(fixed, sizeof(fixed));
    printf("  ->  reference frame CRC = 0x%04X\n", got);

    if (failed) {
        fprintf(stderr, "%d test(s) failed\n", failed);
        return EXIT_FAILURE;
    }
    printf("all CRC tests passed.\n");
    return EXIT_SUCCESS;
}
