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

extern uint16_t ADC_read(uint8_t channel);
extern void ADC_read_isr(uint8_t channel);
extern uint8_t random_adc_seed8(uint8_t channel);
extern uint8_t random_adc_seed16(uint8_t channel);
extern uint32_t random_adc_seed32(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H_ */