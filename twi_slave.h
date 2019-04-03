#ifndef TWI_SLAVE_H
#define TWI_SLAVE_H

#include <stdint.h>

void twi_init(uint8_t ownAddress);

int8_t twi_has_transmitted(void);

int8_t twi_has_received(void);

void twi_close(void);

#endif

