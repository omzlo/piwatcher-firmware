#include <avr/io.h>
#include "adc.h"

void adc_open(void)
{
    ADMUX = (0<<REFS1) | (0<<REFS0); // set Vcc as ref
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  // Enable ADC, prescalar=128 -> 62.5kbps
}

void adc_select_channel(uint8_t channel)
{
    ADMUX = (0<<REFS1) | (0<<REFS0) | (channel & 0x3F);
}

uint16_t adc_read(void)
{
    //Start Single conversion
   ADCSRA|=(1<<ADSC);

   //Wait for conversion to complete
   while(!(ADCSRA & (1<<ADIF)));

   //Clear ADIF by writing one to it
   ADCSRA|=(1<<ADIF);

   return ADC;
}

