
#ifndef AM2302_H_
#define AM2302_H_
 
#include "../Global.h"
#include <avr/io.h>
 
#define DDR_SENSOR   DDRC
#define PORT_SENSOR  PORTC
#define PIN_SENSOR   PINC
#define SENSOR       PC3
 
#ifdef __cplusplus
extern "C"
{
#endif

extern uint8_t am2302_read(uint16_t *humidity, uint16_t *temp);
extern void am2302_init(void);

#ifdef __cplusplus
}
#endif 
 

 
 
#endif /* AM2302_H_ */