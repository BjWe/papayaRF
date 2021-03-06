/*
 * rf4432cppbox.cpp
 *
 * Created: 21.08.2020 08:23:50
 * Author : Bjoern
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdlib.h>
#include <avr/sleep.h>

#include "EasyIO.h"
#include "Global.h"
#include "timedJob/timedJob.h"
#include "UART/UART.h"
#include "SPI/SPI.h"
#include "SI4432/SI4432.h"
#include "XTEA/xtea.h"
#include "ADC/ADC.h"
#include "AM2302/am2302.h"
#include "Watchdog/WatchDog.h"
#include "remoteproto.h"


RemoteProto rp;

#define WAKEUP_DEFINED_MAGIC 0xAB12CD34

volatile uint32_t wakeup_magic __attribute__ ((section (".noinit")));
volatile uint8_t wakeup_rounds __attribute__ ((section (".noinit")));
volatile uint8_t msg_num __attribute__ ((section (".noinit")));

void rf1_shutdown(uint8_t isShutdown){
	SPORT(RF1_SHUTDOWN);
	if(isShutdown){
		SON(RF1_SHUTDOWN);
	} else {
		SOFF(RF1_SHUTDOWN);
	}
}

void rf1_chipselect(uint8_t isSelected){
	SPORT(RF1_CHIPSELECT);
	if(isSelected){
		SOFF(RF1_CHIPSELECT);
	} else {
		SON(RF1_CHIPSELECT);
	}
}

ISR(PCINT0_vect){
	return;
}

ISR(PCINT2_vect){
	return;
}

void wakeup(void){
	wdr();
	PCICR &= ~(1<<PCIE2);
	PCICR &= ~(1<<PCIE1);
	PCICR &= ~(1<<PCIE0);
}

void deep_sleep(void){
		
	PCICR |= (1<<PCIE2);
	PCICR |= (1<<PCIE0);
	
	PCMSK2 |= (1<<PCINT18);
	PCMSK2 |= (1<<PCINT19);
	PCMSK2 |= (1<<PCINT20);
	PCMSK2 |= (1<<PCINT21);	
	PCMSK2 |= (1<<PCINT22);
	PCMSK2 |= (1<<PCINT23);
	PCMSK0 |= (1<<PCINT0);
	//PCMSK0 |= (1<<PCINT1);
		
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

void init(void){
	
	rp = RemoteProto();
	
	watchdog_set_timeout(WATCHDOG_TIMEOUT_1024K);
	//watchdog_set_timeout(WATCHDOG_TIMEOUT_256K);
	watchdog_enable();
	
	uart_init();
	
	if(wakeup_magic != WAKEUP_DEFINED_MAGIC){
		wakeup_magic = WAKEUP_DEFINED_MAGIC;
		wakeup_rounds = 0;
		msg_num = random_adc_seed16(ADC_SEED_CHANNEL);
		
		uart_puts("wdt was not set");
	} else {
		wakeup_rounds++;
		uart_puts("round ");
		uart_print_uint8(wakeup_rounds, 10);
		uart_puts("\r\n");
	}

	timedjob_init();
	
	// Zufall initialisieren
	SPIN(ADC_SEED_PIN);
	SOFF(ADC_SEED_PIN);
	uint32_t seed = random_adc_seed32(ADC_SEED_CHANNEL);
	//uart_put_dw(seed);
	srand(seed);
	
	// Buttons
	SPIN(BTN_1);
	SPIN(BTN_2);
	SPIN(BTN_3);
	SPIN(BTN_4);
	SPIN(BTN_5);
	SPIN(BTN_6);
	SPIN(BTN_7);
	
	SON(BTN_1);
	SON(BTN_2);
	SON(BTN_3);
	SON(BTN_4);
	SON(BTN_5);
	SON(BTN_6);
	SON(BTN_7);
	
	SPIN(RF1_IRQ);
	SON(RF1_IRQ);
	
	
	// RF Initialisieren
	spi_init(SPI_DIV64, SPI_MODE_MASTER, SPI_IRQ_DISABLED, SPI_DATA_MSB, SPI_CPOL_RIFA, SPI_SAMPLE_LEADING);

	sei();
}

uint8_t get_button(void){
	uint8_t result = 0;
	if(ISNSET(BTN_1)){
		result += 0b00000001;
	} 
	if(ISNSET(BTN_2)){
		result += 0b00000010;
	} 
	if(ISNSET(BTN_3)){
		result += 0b00000100;
	} 
	if(ISNSET(BTN_4)){
		result += 0b00001000;
	} 
	if(ISNSET(BTN_5)){
		result += 0b00010000;
	} 
	if(ISNSET(BTN_6)){
		result += 0b00100000;
	} 
	if(ISNSET(BTN_7)){
		result += 0b01000000;
	}
	
	if(result == 0){
		return 0xFF;
	}
	return result;
}


int main(void)
{
	init();
	
	//uart_puts("boot\n");
     
	eeprom_update_dword((uint32_t *)EEPROM_ADDR_SERIALNUM, 0x0d010101);
    
	uint32_t serialnum = eeprom_read_dword((uint32_t *)EEPROM_ADDR_SERIALNUM);

    rp.init(serialnum, EEPROM_ADDR_CRYPTKEY, EEPROM_ADDR_NEXTCODE, RF_BASE_FREQ, &rf1_shutdown, &rf1_chipselect);
	
	
	
    if(wakeup_rounds == 1){
	  //uart_puts("prep");
	  SPORT(DHT_ON);
	  SON(DHT_ON);

	  for(uint8_t wait = 0; wait <= 100; wait++){
	    _delay_ms(100);
		wdr();
	  }	
		
	  uint16_t temp;
	  uint16_t humi;
		
	  am2302_init();
	  //uart_puts("read");
	  uint8_t read_result = am2302_read(&humi, &temp);
	  
	  SPORT(DHT_ON);
	  SOFF(DHT_ON);
	  
	  if(read_result == 0){
		uart_puts("Humi: ");
	    uart_print_uint16(humi, 10);
		uart_puts(" - Temp: ");
	    uart_print_uint16(temp, 10);
		uart_puts("\r\n");
		
		//uint32_t msg = (temp << 16) || humi;
		
		uint16_t msg[4];
		msg[3] = 0;
		msg[2] = 0;
		msg[1] = temp;
		msg[0] = humi;
		
		msg_num++;
		
		rp.openRf(20);
		rp.sendBasicV2Message(TEMPHUMI_VALUE, RETRY_TEMPHUMIMESSAGE_SEND, msg_num, 1, (uint8_t *) &msg);
		rp.closeRf();
		uart_puts("sent\r\n");
	  } else {
		uart_puts("invalid #");
		uart_print_uint8(read_result, 10);
		uart_puts("\r\n");
	  }

	} else if(wakeup_rounds == 2) {
		uint16_t voltage_raw = ADC_read(ADC_MUX_CH_VBG);	
		float voltage_uc = 1126.4; // 0x400 / 1.1
		voltage_uc = voltage_uc / voltage_raw;
		float battery_percent = ((voltage_uc - BATTERY_LEVEL_PERCENT_0) / (BATTERY_LEVEL_PERCENT_100 - BATTERY_LEVEL_PERCENT_0)) * 100;
		if(battery_percent > 100){
		  battery_percent = 100;
		}
		
		uart_puts("V Raw: ");
		uart_print_uint16(voltage_raw, 10);
		uart_puts(" V - Calc: ");
		uart_print_float(voltage_uc, 2);
		uart_puts(" V - Level: ");
		uart_print_float(battery_percent, 2);
		uart_puts(" % \r\n");
		
		
		uint8_t msg[8];
		msg[0] = battery_percent;
		
		msg_num++;
		rp.openRf(20);
		rp.sendBasicV2Message(BATTERY_LEVEL, 5, msg_num, 0, (uint8_t *) &msg);
		rp.closeRf();
		
		
	} else if(wakeup_rounds >= 8){
		
		wakeup_rounds = 0;
	}
	


while(1){
	_delay_ms(15);
	uint8_t key = get_button();
	//uart_putc(key);
	
	if(key != 0xFF){
		_delay_ms(50);
		
		uint8_t key2 = get_button();
		uart_putc(key2);
		if(key2 == 3){
			    uart_puts("rekey\r\n");
				uint8_t newkey[REMOTEPROTO_CRYPTOKEY_SIZE];
				uint32_t newnextcode;
				rp.openRf(20);
				rp.reinitCryptoKeyAndNextcode(&newkey[0], &newnextcode);
				rp.sendRekeyMessage(newkey, newnextcode);
				rp.closeRf();
			
		} else {
			
			uint8_t msg[8];
			memset(&msg, 0, sizeof(msg));
			
			msg[0] = key; 
			
			msg_num++;
			rp.openRf(20);
			rp.sendBasicV2Message(BUTTON_DOWN, RETRY_BUTTONMESSAGE_SEND, msg_num, key, (uint8_t *) &msg);
			
			while(get_button() == key2){
				wdr();
				uart_putc('l');
				_delay_ms(100);
			}
			
			msg_num++;
			rp.sendBasicV2Message(BUTTON_UP, RETRY_BUTTONMESSAGE_SEND, msg_num, key, (uint8_t *) &msg);
			rp.closeRf();
		}
	}

	deep_sleep();
	wakeup();
	
}
	
}

