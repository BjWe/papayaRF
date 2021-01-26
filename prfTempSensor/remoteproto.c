/*
 * remoteproto.c
 *
 * Created: 25.08.2020 14:21:31
 *  Author: Bjoern
 */ 


#include <avr/io.h>
#include <avr/eeprom.h>
#include "ADC/ADC.h"
#include "Global.h"

void rp_reinit_key(void){
	uint32_t key[4];
	for(uint8_t i = 0; i < 4; i++){
		key[i] = random_adc_seed(ADC_SEED_CHANNEL);
	}
	
	
}