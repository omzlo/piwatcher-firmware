#ifndef _TIMER_H_
#define _TIMER_H_

void timer_open(void);

void timer_close(void);

#include <avr/interrupt.h>

#include <util/atomic.h>

#include <registers.h>

extern volatile uint8_t button_press;

__attribute__((always_inline)) inline uint32_t timer_ticks(void) 
{
    
    uint32_t r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        r = out_regs.TICKS;
    }
    return r;    
}

#endif
