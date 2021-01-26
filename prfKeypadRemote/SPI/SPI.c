/*
 * SPI.c
 *
 * Created: 10.02.2013 22:37:46
 *  Author: Bjoern
 */ 
#include <avr/io.h>
#include "../EasyIO.h"
#include "../UART/UART.h"
#include "SPI.h"

void spi_init(uint8_t clk, uint8_t master, uint8_t irq, uint8_t dataorder, uint8_t clkpol, uint8_t clkphase){
	SPIN(PIN_MISO);
	SPORT(PIN_MOSI);
	SPORT(PIN_SCK);
	SPORT(PIN_SS);
	
	SPCR = clk|(1<<SPE)|(irq<<SPIE)|(master<<MSTR)|(dataorder<<DORD)|(clkpol<<CPOL)|(clkphase<<CPHA);
	SPSR = (clk>>2);
}

uint8_t spi_transceive8(uint8_t data){
	// Load data into the buffer
	SPDR = data;
	
	//Wait until transmission complete
	while(!(SPSR & (1<<SPIF) ));
	
	// Return received data
	uint8_t r = SPDR;
	//uart_putc(r);
	return(r);
}

uint16_t spi_transceive16(uint16_t data){
	return (spi_transceive8((uint8_t)(data >> 8)) << 8) | (spi_transceive8((uint8_t)data));
}