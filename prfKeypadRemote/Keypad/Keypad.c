/*
 * Keypad.c
 *
 * Created: 14.01.2013 23:25:33
 *  Author: Bjoern
 */ 

#include <avr/io.h>
#include "Keypad.h"
#include "../Global.h"
#include "../EasyIO.h"
#include <util/delay.h>

//uint8_t keytable[16] = {0x01,0x04,0x07,0xb0,0x02,0x05,0x08,0x00,0x03,0x06,0x09,0xb1,0x0a,0x0b,0x0c,0x0d};

//uint8_t keytable[16] = {0x01,0x02,0x03,0x0a,0x04,0x05,0x06,0x0b,0x07,0x08,0x09,0x0c,0xe1,0x0b,0xe2,0x0d};
uint8_t keytable[16] = {'1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D'};

void keypad_init(void){
	SPORT(KEYPAD_O0);
	SPORT(KEYPAD_O1);
	SPORT(KEYPAD_O2);
	SPORT(KEYPAD_O3);
	
	SON(KEYPAD_O0);
	SON(KEYPAD_O1);
	SON(KEYPAD_O2);
	SON(KEYPAD_O3);
	
	SPIN(KEYPAD_I0);
	SPIN(KEYPAD_I1);
	SPIN(KEYPAD_I2);
	SPIN(KEYPAD_I3);
	
	SON(KEYPAD_I0);
	SON(KEYPAD_I1);
	SON(KEYPAD_I2);
	SON(KEYPAD_I3);
	
}

void keypad_prepare_for_standby(void){
	SPORT(KEYPAD_O0);
	SPORT(KEYPAD_O1);
	SPORT(KEYPAD_O2);
	SPORT(KEYPAD_O3);
	
	SOFF(KEYPAD_O0);
	SOFF(KEYPAD_O1);
	SOFF(KEYPAD_O2);
	SOFF(KEYPAD_O3);
	
	SPIN(KEYPAD_I0);
	SPIN(KEYPAD_I1);
	SPIN(KEYPAD_I2);
	SPIN(KEYPAD_I3);
	
	SON(KEYPAD_I0);
	SON(KEYPAD_I1);
	SON(KEYPAD_I2);
	SON(KEYPAD_I3);
	
}

int8_t keypad_get_row(void){
	if (ISNSET(KEYPAD_I0)) return 0;
	if (ISNSET(KEYPAD_I1)) return 1;
	if (ISNSET(KEYPAD_I2)) return 2;
	if (ISNSET(KEYPAD_I3)) return 3;
	return -1;
}

uint8_t keypad_get_key(void){
	int8_t row;
	SOFF(KEYPAD_O0);
	row = keypad_get_row();
	SON(KEYPAD_O0);
	if(row >= 0) return keytable[row];
	
	SOFF(KEYPAD_O1);
	row = keypad_get_row();
	SON(KEYPAD_O1);
	if(row >= 0) return keytable[4 + row];
	
	SOFF(KEYPAD_O2);
	row = keypad_get_row();
	SON(KEYPAD_O2);
	if(row >= 0) return keytable[8 + row];
		
	SOFF(KEYPAD_O3);
	row = keypad_get_row();
	SON(KEYPAD_O3);
	if(row >= 0) return keytable[12 + row];
	

	return 0xFF;
}

uint8_t keypad_get_key_wait(uint8_t timeout){
	uint8_t loopcounter = 0;
	while(1){
		uint8_t key = keypad_get_key();
		if(key != 0xFF){
			return key;
		}
		
		loopcounter++;
		if((timeout > 0) && (loopcounter > timeout)){
			return 0xFF;
		}
	}
}
