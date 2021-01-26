/*
 * Global.h
 *
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_

#define TESTBOARDX


#ifdef TESTBOARD
#define MHZCPU 16
#else
#define MHZCPU 8
#endif

// CPU-Takt Default 
#ifndef MHZCPU
	#warning "MHZCPU not set. Setting it to 8MHz"
	#define MHZCPU     8
#endif

// Timer Prescaler
#define TIMER1_PRESCALER  0x02

// Setzen der Timer auf einen 10ms Interrupt
#if MHZCPU == 16
	#define F_CPU             16000000UL
	#define TIMER1_COMPA_VAL  0x4E1F
#elif MHZCPU == 8
	#define F_CPU             8000000UL
	#define TIMER1_COMPA_VAL  0x270F
#else
	#error "Unknown MHZCPU"
#endif

#define RF_BASE_FREQ  433.8E6

#define UART_BAUD  9600UL

#ifdef TESTBOARD
#error "x"
#define KEYPAD_O0  C,0
#define KEYPAD_O1  C,1
#define KEYPAD_O2  C,2
#define KEYPAD_O3  C,3

#define KEYPAD_I0  B,0
#define KEYPAD_I1  D,5
#define KEYPAD_I2  D,6
#define KEYPAD_I3  D,7

#define RF1_SHUTDOWN   D,2
#define RF1_CHIPSELECT B,2
#define RF1_IRQ        D,3

#define ADC_SEED_PIN     C,5
#define ADC_SEED_CHANNEL 5

#else

#define CONFIG_BTN C,2

#define KEYPAD_O0  D,2
#define KEYPAD_O1  D,3
#define KEYPAD_O2  D,4
#define KEYPAD_O3  D,5

#define KEYPAD_I0  D,6
#define KEYPAD_I1  D,7
#define KEYPAD_I2  B,0
#define KEYPAD_I3  B,1

#define RF1_SHUTDOWN   C,1
#define RF1_CHIPSELECT B,2
#define RF1_IRQ        C,0

#define ADC_SEED_PIN     C,5
#define ADC_SEED_CHANNEL 5

#endif

#define EEPROM_ADDR_SERIALNUM 0x04
#define EEPROM_ADDR_CRYPTKEY  0x10
#define EEPROM_LEN_CRYPTKEY   16
#define EEPROM_ADDR_NEXTCODE  0x20
#define EEPROM_LEN_NEXTCODE   4

#include <util/delay.h>

static inline void Delay_ms(uint8_t d){
	uint8_t i;
	for(i = 0; i < d; i++){
		_delay_ms(1);
	}
}

static inline void Delay_us(uint_fast16_t d){
	_delay_us(d);
}


#endif /* GLOBAL_H_ */