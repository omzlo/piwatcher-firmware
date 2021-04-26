#include "registers.h"
#include <avr/eeprom.h>

registers_t out_regs;
registers_t in_regs;

#define DWDT1 ((uint8_t *)0)
#define DWDT2 ((uint8_t *)1)
#define DRBT1  ((uint16_t *)2)
#define DRBT2  ((uint16_t *)4)

#define BIT_LED 1

void registers_reset(void)
{
    /* get default watchdog delay */
    uint8_t dwdt1 = eeprom_read_byte(DWDT1);
    uint8_t dwdt2 = eeprom_read_byte(DWDT2);

    if ((dwdt1^dwdt2)==0xFF)
        out_regs.DEFAULT_WATCHDOG = dwdt1;
    else
        out_regs.DEFAULT_WATCHDOG = 0;

    in_regs.DEFAULT_WATCHDOG = out_regs.DEFAULT_WATCHDOG;

    /* get default reboot delay */
    uint16_t drbt1 = eeprom_read_word(DRBT1);
    uint16_t drbt2 = eeprom_read_word(DRBT2);

    if ((drbt1^drbt2)==0xFFFF)
        out_regs.DEFAULT_REBOOT = drbt1;
    else
        out_regs.DEFAULT_REBOOT = 0;
    
    in_regs.DEFAULT_REBOOT = out_regs.DEFAULT_REBOOT;

    out_regs.STATUS     = 0;
    out_regs.WATCHDOG   = out_regs.DEFAULT_WATCHDOG;
    out_regs.REBOOT     = out_regs.DEFAULT_REBOOT;

    in_regs.STATUS      = 0;
    in_regs.WATCHDOG    = out_regs.DEFAULT_WATCHDOG;
    in_regs.REBOOT      = out_regs.DEFAULT_REBOOT;

    out_regs.VERSION = 2;
}

void registers_sync(void)
{
   out_regs.STATUS &= ~(in_regs.STATUS);
   in_regs.STATUS = 0;
   out_regs.WATCHDOG = in_regs.WATCHDOG;
   out_regs.REBOOT = in_regs.REBOOT;

   if (in_regs.VERSION==0x81)
   {
        PORTB &= ~(1<<BIT_LED);
        in_regs.VERSION = 0;        
   }
   
   if (in_regs.VERSION==0x82)
   {
        PORTB |= (1<<BIT_LED);
        in_regs.VERSION = 0;        
   }

   if (in_regs.DEFAULT_WATCHDOG!=out_regs.DEFAULT_WATCHDOG)
   {
        eeprom_write_byte(DWDT1, in_regs.DEFAULT_WATCHDOG);
        eeprom_write_byte(DWDT2, in_regs.DEFAULT_WATCHDOG^0xFF);
        out_regs.DEFAULT_WATCHDOG = in_regs.DEFAULT_WATCHDOG;
   }

   if (in_regs.DEFAULT_REBOOT!=out_regs.DEFAULT_REBOOT)
   {
        eeprom_write_word(DRBT1, in_regs.DEFAULT_REBOOT);
        eeprom_write_word(DRBT2, in_regs.DEFAULT_REBOOT^0xFFFF);
        out_regs.DEFAULT_REBOOT = in_regs.DEFAULT_REBOOT;
   }
}

void registers_clear_defaults(void)
{
    in_regs.DEFAULT_WATCHDOG = 0;
    in_regs.DEFAULT_REBOOT = 0;
    registers_sync();
}

