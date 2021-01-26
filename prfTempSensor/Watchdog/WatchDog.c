/*
 * WatchDog.c
 *
 * Created: 26.12.2014 11:10:37
 *  Author: Bjoern
 */ 

#include <avr/io.h>
#include "WatchDog.h"

#define WATCHDOG_CHANGE_ENABLE   WATCHDOG_CONTROL_REG |= (1<<WDCE)
//#define MCU_STATUSREG            MCUCSR

void watchdog_enable(void){
	wdr();
	WATCHDOG_CHANGE_ENABLE;
	WATCHDOG_CONTROL_REG |= (1<<WDE);
}

void watchdog_disable(void){
	wdr();
	WATCHDOG_CHANGE_ENABLE;
	WATCHDOG_CONTROL_REG &= ~(1<<WDE);
}

void watchdog_set_timeout(uint8_t timeout){
	
	
	uint8_t ctrlreg = WATCHDOG_CONTROL_REG;
	#if defined(WDP3)
	  ctrlreg &= ~(WDP3);
	  ctrlreg |= ((timeout & 0b00001000) >> 3) << WDP3;
	#endif
	
	ctrlreg &= ~((1<<WDP2) | (1<<WDP1) | (1<<WDP0));
	ctrlreg |= ((timeout & 0b00000100) >> 2) << WDP2;
	ctrlreg |= ((timeout & 0b00000010) >> 1) << WDP1;
	ctrlreg |= ((timeout & 0b00000001)     ) << WDP0;
	
	wdr();
	WATCHDOG_CHANGE_ENABLE;
	WATCHDOG_CONTROL_REG = ctrlreg;	
}

uint8_t watchdog_occured(void){
	return (WATCHDOG_STATUS_REG & (1<<WDRF)) >> WDRF;
}

void watchdog_clear_flag(void){
	WATCHDOG_STATUS_REG &= ~(1<<WDRF);
}