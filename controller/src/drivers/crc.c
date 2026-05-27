#include "crc.h"

uint16_t crc16_ccitt(const void *data, size_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
