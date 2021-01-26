#include "UART.h"
#include "../Global.h"


#ifdef UCSR0A
void uart_init (void)
{
	uint16_t ubrr = (uint16_t) ((uint32_t) F_CPU/(16*UART_BAUD) - 1);
	
	UBRR0H = (uint8_t) (ubrr>>8);
	UBRR0L = (uint8_t) (ubrr);
	
	// UART Receiver und Transmitter anschalten
	// Data mode 8N1, asynchron
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	// Flush Receive-Buffer (entfernen evtl. vorhandener ungültiger Werte)
	do
	{
		UDR0;
	}
	while (UCSR0A & (1 << RXC0));
}

uint8_t uart_getc_wait_timeout (uint8_t *data, uint8_t timeout){
	for(uint8_t i = 0; i < timeout; i++){
		if(!uart_data_ready()){
			_delay_ms(1);
			} else {
			*data = UDR0;
			return 1;
		}
	}
	
	return 0;
}

void uart_puts (char *s)
{
	while (*s)
	{
		uart_putc(*s);
		s++;
	}
}
#else

void uart_init (void)
{
	uint16_t ubrr = (uint16_t) ((uint32_t) F_CPU/(16*UART_BAUD) - 1);
	
	UBRRH = (uint8_t) (ubrr>>8);
	UBRRL = (uint8_t) (ubrr);
	
	// UART Receiver und Transmitter anschalten
	// Data mode 8N1, asynchron
	UCSRB = (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);

	// Flush Receive-Buffer (entfernen evtl. vorhandener ungültiger Werte)
	do
	{
		UDR;
	}
	while (UCSRA & (1 << RXC));
}

uint8_t uart_getc_wait_timeout (uint8_t *data, uint8_t timeout){
	for(uint8_t i = 0; i < timeout; i++){
		if(!uart_data_ready()){
			Delay_ms(1);
		} else {
			*data = UDR;
			return 1;
		}
	}
	
	return 0;
}

void uart_puts (char *s)
{
	while (*s)
	{
		uart_putc(*s);
		s++;
	}
}

#endif