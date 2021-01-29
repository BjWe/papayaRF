#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdlib.h>
#include "../Global.h"

#define UBRR_VAL (F_CPU/(UART_BAUD*16)-1)   
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/UART_BAUD) // Fehler in Promille, 1000 = kein Fehler.

#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
#error Systematischer Fehler der Baudrate groesser 1% und damit zu hoch!
#endif

#ifdef __cplusplus
extern "C"
{
#endif

extern void uart_init (void);
extern void uart_puts (char *s);
extern uint8_t uart_getc_wait_timeout (uint8_t *data, uint8_t timeout);

#ifdef UCSRA

static inline int uart_putc (uint8_t c){
	// Warten, bis UDR bereit ist für einen neuen Wert
	while (!(UCSRA & (1 << UDRE)))
	;

	// UDR Schreiben startet die Übertragung
	UDR = c;

	return 1;
}

static inline uint8_t uart_data_ready(void)
{
	return UCSRA & (1 << RXC);
}

static inline uint8_t uart_getc_wait (void)
{
	// Warten, bis etwas empfangen wird
	while (!(UCSRA & (1 << RXC)))
	;

	// Das empfangene Zeichen zurückliefern
	return UDR;
}

static inline int uart_getc_nowait (void)
{
	// Liefer das empfangene Zeichen, falls etwas empfangen wurde; -1 sonst
	return (UCSRA & (1 << RXC)) ? (int) UDR : -1;
}

#else

static inline int uart_putc (const uint8_t c)
{
	// Warten, bis UDR bereit ist für einen neuen Wert
	while (!(UCSR0A & (1 << UDRE0)))
	;

	// UDR Schreiben startet die Übertragung
	UDR0 = c;

	return 1;
}

static inline uint8_t uart_data_ready(void)
{
	return UCSR0A & (1 << RXC0);
}

static inline uint8_t uart_getc_wait (void)
{
	// Warten, bis etwas empfangen wird
	while (!(UCSR0A & (1 << RXC0)))
	;

	// Das empfangene Zeichen zurückliefern
	return UDR0;
}


static inline int uart_getc_nowait (void)
{
	// Liefer das empfangene Zeichen, falls etwas empfangen wurde; -1 sonst
	return (UCSR0A & (1 << RXC0)) ? (int) UDR0 : -1;
}


#endif


static inline void uart_print_uint8(uint8_t i, uint8_t radix){
  char out[4];
  itoa(i, out, radix);
  uart_puts(out);
}

static inline void uart_print_uint16(uint16_t i, uint8_t radix){
	char out[6];
	itoa(i, out, radix);
	uart_puts(out);
}

static inline void uart_print_uint32(uint32_t i, uint8_t radix){
	char out[14];
	itoa(i, out, radix);
	uart_puts(out);
}

static inline void uart_print_float(float f, uint8_t precision){
	char out[14];
	dtostrf(f, 10, precision, out);
	uart_puts(out);
}


static inline void uart_put_w(const uint16_t w){
	uart_putc((uint8_t)(w>>8));
	uart_putc((uint8_t)w);
}

static inline void uart_put_dw(const uint32_t w){
	uart_putc((uint8_t)(w>>24));
	uart_putc((uint8_t)(w>>16));
	uart_putc((uint8_t)(w>>8));
	uart_putc((uint8_t)w);
}

#ifdef __cplusplus
}
#endif

#endif /* UART_H  */