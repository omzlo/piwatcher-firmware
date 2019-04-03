#ifndef _ADC_H_
#define _ADC_H_

void adc_open(void);

void adc_select_channel(uint8_t channel);

uint16_t adc_read(void);

#endif
