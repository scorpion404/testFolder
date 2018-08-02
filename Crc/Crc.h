#ifndef __CRC_H
#define __CRC_H
#include "stdint.h"

uint16_t Crc_16(uint8_t Array[], uint16_t arraysize);
uint32_t Crc_32(uint8_t Array[], uint16_t arraysize);
uint8_t Crc_8(uint8_t Array[], uint16_t arraysize);

#endif
