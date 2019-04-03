#include "registers.h"

registers_t out_regs;
registers_t in_regs;

void registers_reset(void)
{
    out_regs.STATUS = 0;
    out_regs.WATCHDOG = 0;
    out_regs.REBOOT = 0;

    in_regs.STATUS = 0;
    in_regs.WATCHDOG = 0;
    in_regs.REBOOT = 0;
}

void registers_sync(void)
{
   out_regs.STATUS &= ~(in_regs.STATUS);
   in_regs.STATUS = 0;
   out_regs.WATCHDOG = in_regs.WATCHDOG;
   out_regs.REBOOT = in_regs.REBOOT;
}

