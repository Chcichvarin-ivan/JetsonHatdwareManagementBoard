/** crc8.h — CRC-8/SMBus (poly 0x07, init 0x00, no reflection, xorout 0x00). */
#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>
#include <stddef.h>

uint8_t crc8_smbus(const uint8_t *data, size_t len);

#endif /* CRC8_H */
