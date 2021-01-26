/*
 * Global.h
 *
 */ 


#ifndef GLOBAL_H_
#define GLOBAL_H_

#define RETRY_BUTTONMESSAGE_SEND    (uint8_t) 3
#define RETRY_TEMPHUMIMESSAGE_SEND  (uint8_t) 10

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


#define BTN_1  D,2
#define BTN_2  D,3
#define BTN_3  D,4
#define BTN_4  D,5
#define BTN_5  D,6
#define BTN_6  D,7
#define BTN_7  B,0


#define RF1_SHUTDOWN   C,1
#define RF1_CHIPSELECT B,2
#define RF1_IRQ        C,0

#define DHT_ON   C,2

#define ADC_SEED_PIN     C,5
#define ADC_SEED_CHANNEL 5



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