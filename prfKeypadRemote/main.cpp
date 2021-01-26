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
#include "Keypad/Keypad.h"
#include "SI4432/SI4432.h"
#include "XTEA/xtea.h"
#include "ADC/ADC.h"

#include "remoteproto.h"

uint8_t cryptkey[16] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6};

//SI4432 rf1;

RemoteProto rp;

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
	PCICR &= ~(1<<PCIE2);
	PCICR &= ~(1<<PCIE1);
	PCICR &= ~(1<<PCIE0);
	
	keypad_init();
}

void deep_sleep(void){
	keypad_prepare_for_standby();
		
	PCICR |= (1<<PCIE2);
	PCICR |= (1<<PCIE0);
		
	PCMSK2 |= (1<<PCINT22);
	PCMSK2 |= (1<<PCINT23);
	PCMSK0 |= (1<<PCINT0);
	PCMSK0 |= (1<<PCINT1);
		
		
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

void init(void){
	uart_init();

	
	timedjob_init();
	
	keypad_init();
	
	// Zufall initialisieren
	SPIN(ADC_SEED_PIN);
	SOFF(ADC_SEED_PIN);
	uint32_t seed = random_adc_seed32(ADC_SEED_CHANNEL);
	//uart_put_dw(seed);
	srand(seed);
	
	// Conf Butoon
	SPIN(CONFIG_BTN);
	SON(CONFIG_BTN);
	
	SPIN(RF1_IRQ);
	SON(RF1_IRQ);
	
	
	// RF Initialisieren
	spi_init(SPI_DIV64, SPI_MODE_MASTER, SPI_IRQ_DISABLED, SPI_DATA_MSB, SPI_CPOL_RIFA, SPI_SAMPLE_LEADING);

	sei();
}

int main(void)
{
	init();
	
	uart_puts("boot\n");

    //eeprom_update_dword((uint32_t *)EEPROM_ADDR_SERIALNUM, 0x11111111);
	uint32_t serialnum = eeprom_read_dword((uint32_t *)EEPROM_ADDR_SERIALNUM);

    rp.init(serialnum, EEPROM_ADDR_CRYPTKEY, EEPROM_ADDR_NEXTCODE, RF_BASE_FREQ, &rf1_shutdown, &rf1_chipselect);

while(1){
	_delay_ms(15);
	uint8_t key = keypad_get_key();
	
	if(key != 0xFF){
		if(ISNSET(CONFIG_BTN)){
			if(key == '1'){
				//_delay_ms(10);
				//uart_puts("rekey");
				uint8_t newkey[REMOTEPROTO_CRYPTOKEY_SIZE];
				uint32_t newnextcode;
				rp.openRf(20);
				rp.reinitCryptoKeyAndNextcode(&newkey[0], &newnextcode);
				rp.sendRekeyMessage(newkey, newnextcode);
				rp.closeRf();
			}
		} else {
			//uart_putc(key);
			//_delay_ms(10);
			uint16_t msg;
			memset(&msg, key, sizeof(msg));
			rp.openRf(5);
			rp.sendBasicMessage(BUTTON_DOWN, msg);
			
			while(keypad_get_key() == key){
				_delay_ms(100);
			}
			
			rp.sendBasicMessage(BUTTON_UP, msg);
			rp.closeRf();
		}
	}

	deep_sleep();
	wakeup();
	
}
	/*
	uint8_t lastkey = 0xFF;
	
	while (1){
		uart_putc(0xA0);
		uint8_t key = keypad_get_key();
		uart_putc(key);
		
		if((key != 0xFF) && (lastkey != key)){
			lastkey = key;		
			
			uint16_t msg;
			memset(&msg, key, sizeof(msg));
			rp.openRf(20);
			rp.sendBasicMessage(BUTTON_PRESSED, msg);
			rp.closeRf();
			
		}
		
		uart_putc(0xAA);
		_delay_ms(100);

		
	}
	*/
}

