/*
 * ADC.c
 */ 

#include <avr/io.h>

uint16_t ADC_read(uint8_t channel){
	uint16_t result = 0xFFFF;
	// AVCC benutzen 7:6
	// Right Adjust 5
	// Channel 4:0
	
	ADMUX = 0b01000000 | (0b00001111 & channel);
	// Aktivieren 7
	// Start Conversion 6
	// Free running 5
	// Interupt 4:3
	// Prescaler 2:0
	ADCSRA = 0b11000110;
	

	// Initialberechnung
	while (ADCSRA & (1<<ADSC))
	asm("nop");
	
	// Wert berechnen lassen
	ADCSRA |= 1<<ADSC;

	while (ADCSRA & (1<<ADSC))
	asm("nop");
	

	// Wert lesen. ADCL zuerst!
	result = ADCL;
	result += (ADCH << 8);
	
	// ADC deaktivieren
	ADCSRA = 0;
	
	return result;
}

void ADC_read_isr(uint8_t channel){

	// AVCC benutzen 7:6
	// Right Adjust 5
	// Channel 4:0
	
	ADMUX = 0b01000000 | (0b00001111 & channel);
	// Aktivieren 7
	// Start Conversion 6
	// Free running 5
	// Interupt 4:3
	// Prescaler 2:0
	ADCSRA = 0b11011011;

}

uint8_t random_adc_seed8(uint8_t channel){
	uint8_t seed = 0;
	for(uint8_t i = 0; i < 8; i++){
		uint8_t measure = ADC_read(channel);
		seed = (seed << 1) | (measure & 1);
	}
	return seed;
}

uint32_t random_adc_seed32(uint8_t channel){
	uint32_t seed = 0;
	for(uint8_t i = 0; i < 32; i++){
		uint8_t measure = ADC_read(channel);
		seed = (seed << 1) | (measure & 1);
	}
	return seed;
}