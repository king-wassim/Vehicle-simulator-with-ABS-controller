#ifndef ABS_CRC_H
#define ABS_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CRC-16/CCITT-FALSE (AUTOSAR E2E).
 *   polynomial 0x1021, initial 0xFFFF, refin/refout=false, xorout 0x0000.
 *   Known-answer test: crc16_ccitt("123456789", 9) == 0x29B1.
 *
 * Implementation: bytewise table-free shift-register. Fast enough on a
 * Cortex-M and trivial to audit; on real automotive ECUs we would swap for
 * a 256-entry table or HW accelerator without changing this API.
 */
uint16_t crc16_ccitt(const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* ABS_CRC_H */
