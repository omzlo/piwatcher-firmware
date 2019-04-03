#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "registers.h"

volatile uint8_t button_press;

void timer_open(void)
{
    out_regs.TICKS = 0;
    button_press = 0;

    TCCR1 = 0;
     // set up timer with prescaler = 8192, CTC mode
    TCCR1 = (1 << CS13) | (1 << CS12) | (1 << CS11) | (1 << CTC1);

    // initialize counter
    TCNT1 = 0;

    // 8 000 000 / (8192 * 39) -> approx 25 ticks per second 
    // We substract 1 to 39, since we count from 0 to 38 to get 39 ticks
    OCR1A = 38;
    OCR1C = 38;
  
    // enable compare interrupt
    TIMSK |= (1 << OCIE1A);
}

void timer_close(void)
{
    TCCR1 = 0;
}

static uint8_t state;

ISR ( TIMER1_COMPA_vect )
{
    out_regs.TICKS++;
    state = ((state<<1) | ((PINB >> 4) & 1))&0x0F;
    button_press = (state==0x00);
}

extern uint32_t timer_ticks(void);


