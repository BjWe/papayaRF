
#include "crc.h"

#include "Arduino.h"

uint8_t crc8maxim(void *ptr, uint8_t len) {
  uint8_t crc = 0;

  uint8_t *addr = (uint8_t *)ptr;

  for (uint8_t i = 0; i < len; i++) {
    uint8_t inbyte = addr[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C;

      inbyte >>= 1;
    }
  }
  return crc;
}
