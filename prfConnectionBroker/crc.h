
#ifndef CRC_H_
#define CRC_H_

#include "Arduino.h"

#define CRC8INIT 0x00
#define CRC8POLY 0x31  // = X^8+X^5+X^4+X^0

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t crc8maxim(void *ptr, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H_ */
