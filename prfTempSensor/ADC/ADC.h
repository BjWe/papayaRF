/*
 * ADC.h
 *
 * Created: 05.10.2016 15:36:04
 *  Author: bjoern
 */ 


#ifndef ADC_H_
#define ADC_H_

#ifdef __cplusplus
extern "C"
{
#endif


// http://ww1.microchip.com/downloads/en/Appnotes/00002447A.pdf
#define ADC_MUX_CH0    0b0000
#define ADC_MUX_CH1    0b0001
#define ADC_MUX_CH2    0b0010
#define ADC_MUX_CH3    0b0011
#define ADC_MUX_CH4    0b0100
#define ADC_MUX_CH5    0b0101
#define ADC_MUX_CH6    0b0110
#define ADC_MUX_CH7    0b0111
#define ADC_MUX_CH_TEM 0b1000
#define ADC_MUX_CH_VBG 0b1110
#define ADC_MUX_CH_GND 0b1111


extern uint16_t ADC_read(uint8_t channel);
extern void ADC_read_isr(uint8_t channel);
extern uint8_t random_adc_seed8(uint8_t channel);
extern uint8_t random_adc_seed16(uint8_t channel);
extern uint32_t random_adc_seed32(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H_ */