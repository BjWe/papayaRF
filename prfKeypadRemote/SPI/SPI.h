/*
 * SPI.h
 *
 * Created: 10.02.2013 22:31:24
 *  Author: Bjoern
 */ 


#ifndef SPI_H_
#define SPI_H_

#define SPI_DIV4   0x00
#define SPI_DIV16  0x01
#define SPI_DIV64  0x02
#define SPI_DIV128 0x03
#define SPI_DIV2   0x04
#define SPI_DIV8   0x05
#define SPI_DIV32  0x06
#define SPI_DIV642 0x07 /* nicht benötigt */

#define SPI_IRQ_ENABLED     1
#define SPI_IRQ_DISABLED    0

#define SPI_DATA_MSB        0
#define SPI_DATA_LSB        1

#define SPI_MODE_SLAVE      0
#define SPI_MODE_MASTER     1

#define SPI_CPOL_RIFA       0
#define SPI_CPOL_FARI       1

#define SPI_SAMPLE_LEADING  0
#define SPI_SAMPLE_TRAILING 1


#if   defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__)
#define PIN_MOSI B,5
#define PIN_MISO B,6
#define PIN_SCK  B,7
#define PIN_SS   B,4
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__)
#define PIN_MOSI B,3
#define PIN_MISO B,4
#define PIN_SCK  B,5
#define PIN_SS   B,2
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega168PA__) || defined(__AVR_ATmega328P__)
#define PIN_MOSI B,3
#define PIN_MISO B,4
#define PIN_SCK  B,5
#define PIN_SS   B,2
#elif defined(__AVR_ATmega128A__) || defined(__AVR_ATmega128__)
#define PIN_MOSI B,2
#define PIN_MISO B,3
#define PIN_SCK  B,1
#define PIN_SS   B,0
#else
#error SPI unknowm MCU
#endif

#ifdef __cplusplus
extern "C"
{
#endif

extern void spi_init(uint8_t clk, uint8_t master, uint8_t irq, uint8_t dataorder, uint8_t clkpol, uint8_t clkphase);
extern uint8_t spi_transceive8(uint8_t data);
extern uint16_t spi_transceive16(uint16_t data);

#ifdef __cplusplus
}
#endif




#endif /* SPI_H_ */