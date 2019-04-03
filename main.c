#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "timer.h"
#include "twi_slave.h"
#include <avr/sleep.h>

/*
 * PB0  I2C (SDA)
 * PB1  LED
 * PB2  I2C (SCL)
 * PB3  DRIVE
 * PB4  BUTTON
 * PB5  RESET
 */

#define BIT_LED 1
#define BIT_DRIVE 3
#define BIT_BUTTON  4


#define INVERTED_LOGIC
#ifdef POSITIVE_LOGIC
    #define SWITCH_ON() (PORTB |= (1<<BIT_DRIVE))
    #define SWITCH_OFF() (PORTB &= ~(1<<BIT_DRIVE))
#endif

#ifdef INVERTED_LOGIC
    #define SWITCH_ON() (PORTB &= ~(1<<BIT_DRIVE))
    #define SWITCH_OFF() (PORTB |= (1<<BIT_DRIVE))
#endif

#define BUTTON_STATE_NONE           0
#define BUTTON_STATE_PRESS_START    1
#define BUTTON_STATE_PRESS_SHORT    2
#define BUTTON_STATE_PRESS_LONG     3

static void gpio_init(void)
{
    DDRB = (1<<BIT_LED) | (1<<BIT_DRIVE);

    SWITCH_ON();
    //PORTB |= (1<<BIT_DRIVE);


    //PCMSK = (1<<PCINT4); // PCINT4 is button
    //GIMSK = (1<<PCIE);     // enable pin change interrupt
}

ISR (PCINT0_vect)
{
    /* void */
}


static void slow_blink(void) 
{
    if ((((timer_ticks()))&0x38)==0x30)
        PORTB |= (1<<BIT_LED);
    else
        PORTB &= ~(1<<BIT_LED);
}

static void shutdown(void)
{
    PORTB &= ~(1<<BIT_LED);
    SWITCH_OFF();
    _delay_ms(100);
    timer_close();

    GIMSK = (1<<PCIE);
    PCMSK = (1<<PCINT4);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    
    GIMSK = 0;
    PORTB |= (1<<BIT_LED);

    timer_open();
    _delay_ms(500);
    while (button_press);

    SWITCH_ON();
    twi_init(0x62);
    registers_reset();
    out_regs.STATUS = REG_STATUS_BOOT_BUTTON;
}

static void reboot(uint32_t wait_until)
{
    uint32_t start = timer_ticks();
    
    SWITCH_OFF();
    set_sleep_mode(SLEEP_MODE_IDLE);
    do 
    {
        sleep_mode();
        slow_blink();
    } 
    while ((timer_ticks()-start<wait_until) && (!button_press));
    PORTB |= (1<<BIT_LED);
    SWITCH_ON();
    registers_reset();
    if (button_press)
    {
        _delay_ms(500);
        while (button_press);
        out_regs.STATUS = REG_STATUS_BOOT_BUTTON;
    }
    else
    {
        out_regs.STATUS = REG_STATUS_BOOT_TIMER;
    }
}

int main() 
{
    uint8_t button_state= BUTTON_STATE_NONE;
    uint32_t start_timer = 0;
    uint32_t button_start = 0;
    uint32_t now;
    uint32_t interval;


    /* power reduction efforts */
    ACSR |= (1<<ACD);   // Dissable analog comparator
    PRR |= (1<<PRTIM0); // Dissable timer/counter 0;
    PRR |= (1<<PRADC);  // Dissable ADC

    gpio_init();
    timer_open();
    twi_init(0x62);
    
    _delay_ms(250);
    sei();



    PORTB |= (1<<BIT_LED);

    for (;;)
    {
        now = timer_ticks();
        
        if (twi_has_received()) // there has been a change
        {
            registers_sync();
            start_timer = now;
        }

        switch (button_state) {
            case BUTTON_STATE_NONE:
                if (button_press)
                {
                    button_start = now;
                    button_state = BUTTON_STATE_PRESS_START;
                }
                break;
            case BUTTON_STATE_PRESS_START:
                interval = now - button_start;
                if (button_press)
                {
                    if (interval > 75) // 3 seconds = 3*25
                    {
                        while (button_press)
                        {
                            PORTB ^= (1<<BIT_LED);
                            _delay_ms(125); 
                        }
                        button_state = BUTTON_STATE_NONE;
                        shutdown();
                    }
                }
                else // !button_press
                {
                    out_regs.STATUS |= REG_STATUS_BUTTON;
                    button_state = BUTTON_STATE_NONE;
                }
                break;
        }

        if (out_regs.WATCHDOG!=0)
        {
            if (twi_has_transmitted()) {
                start_timer = now;
            }
            
            interval = ((uint32_t)out_regs.WATCHDOG)*25;

            if ((now-start_timer)>interval)
            {
                /* WATCHDOG MODE (REBOOT ON LOSS OF ACTIVITY) */
                if (out_regs.REBOOT!=0)
                {
                    reboot((uint32_t)out_regs.REBOOT*50);
                }
                else
                /* SHUTDOWN MODE (HALT ON LOSS OF ACTIVITY) */
                {
                    shutdown();
                }
            }
        }
    }
}
