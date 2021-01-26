/*
 * WatchDog.h
 *
 * Created: 26.12.2014 11:10:54
 *  Author: Bjoern
 */ 


#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <avr/io.h>
#include <stdint.h>

#if defined(__AVR_ATmega644__) || defined(__AVR_ATmega328P__)
  #define WATCHDOG_TIMEOUT_2K    0b00000000
  #define WATCHDOG_TIMEOUT_4K    0b00000001
  #define WATCHDOG_TIMEOUT_8K    0b00000010
  #define WATCHDOG_TIMEOUT_16K   0b00000011
  #define WATCHDOG_TIMEOUT_32K   0b00000100
  #define WATCHDOG_TIMEOUT_64K   0b00000101
  #define WATCHDOG_TIMEOUT_128K  0b00000110
  #define WATCHDOG_TIMEOUT_256K  0b00000111 
  #define WATCHDOG_TIMEOUT_512K  0b00001000 
  #define WATCHDOG_TIMEOUT_1024K 0b00001001 
#elif defined(__AVR_ATmega8__)  \
   || defined(__AVR_ATmega128__)
  #define WATCHDOG_TIMEOUT_16K   0b00000000
  #define WATCHDOG_TIMEOUT_32K   0b00000001
  #define WATCHDOG_TIMEOUT_64K   0b00000010
  #define WATCHDOG_TIMEOUT_128K  0b00000011
  #define WATCHDOG_TIMEOUT_256K  0b00000100
  #define WATCHDOG_TIMEOUT_512K  0b00000101
  #define WATCHDOG_TIMEOUT_1024K 0b00000110
  #define WATCHDOG_TIMEOUT_2048K 0b00000111 
#else
  #error "unkown mcu"
#endif

#define wdr() __asm__ __volatile__ ("wdr")

#if defined(WDTCSR)
  #define WATCHDOG_CONTROL_REG     WDTCSR
#elif defined(WDTCR)
  #define WATCHDOG_CONTROL_REG     WDTCR
#else
  #define WATCHDOG_CONTROL_REG     WDT
#endif

#if defined(MCUCSR)
  #define WATCHDOG_STATUS_REG      MCUCSR
#elif defined(MCUCR)
  #define WATCHDOG_STATUS_REG      MCUSR
#else
  #error "no wdt status reg"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

extern void watchdog_enable(void);
extern void watchdog_disable(void);
extern void watchdog_set_timeout(uint8_t timeout);

extern uint8_t watchdog_occured(void);
extern void watchdog_clear_flag(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H_ */