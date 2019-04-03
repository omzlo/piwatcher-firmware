#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>

typedef struct {
    volatile uint8_t STATUS;
        // R+W
        // Get the state of the device
        // bit 7: 1 if button short press, reset by setting 0
        // bit 6: 1 if rebooted from timer
        // bit 5: 1 if rebooted from button

    volatile uint8_t WATCHDOG;
        // R+W
        // if 0, dissable watchdog
        // else number of ticks * 25 before shutdown, reset by I2C communication

    volatile uint16_t REBOOT;
        // R+W
        // if 0, do not reboot
        // else (number of ticks * 2) until reboot

    volatile uint32_t TICKS;
        // R only
        // current clock value

} __attribute__ ((__packed__)) registers_t;

extern registers_t out_regs;
extern registers_t in_regs;

#define REG_STATUS_BUTTON       0x80
#define REG_STATUS_BOOT_TIMER   0x40
#define REG_STATUS_BOOT_BUTTON  0x20

void registers_reset(void);

void registers_sync(void);

#endif
