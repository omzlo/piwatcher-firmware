/********************************************************************************

USI TWI Slave driver.

Created by Donald R. Blake. donblake at worldnet.att.net
Adapted by Jochen Toppe, jochen.toppe at jtoee.com
Adapted by Alain Pannetrat, omzlo.com

---------------------------------------------------------------------------------

Created from Atmel source files for Application Note AVR312: Using the USI Module
as an I2C slave.

This program is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

---------------------------------------------------------------------------------

Change Activity:

    Date       Description
   ------      -------------
  16 Mar 2007  Created.
  27 Mar 2007  Added support for ATtiny261, 461 and 861.
  26 Apr 2007  Fixed ACK of slave address on a read.
  04 Jul 2007  Fixed USISIF in ATtiny45 def
  12 Dev 2009  Added callback functions for data requests
  06 Feb 2015  Minor change to allow mutli-byte requestFrom() from master.
  10 Feb 2015  Simplied RX/TX buffer code and allowed use of full buffer.
  12 Dec 2016  Added support for ATtiny167
  06 Deb 2019  Changed to a register based approach, removed unused code.
  

********************************************************************************/


/********************************************************************************
                                    includes
********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "twi_slave.h"
#include "registers.h"

/********************************************************************************
                            device dependent defines
********************************************************************************/

#if defined( __AVR_ATtiny167__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATtiny2313__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB5
#  define PORT_USI_SCL        PB7
#  define PIN_USI_SDA         PINB5
#  define PIN_USI_SCL         PINB7
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined(__AVR_ATtiny84__) | \
     defined(__AVR_ATtiny44__)
#  define DDR_USI             DDRA
#  define PORT_USI            PORTA
#  define PIN_USI             PINA
#  define PORT_USI_SDA        PORTA6
#  define PORT_USI_SCL        PORTA4
#  define PIN_USI_SDA         PINA6
#  define PIN_USI_SCL         PINA4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny25__ ) | \
     defined( __AVR_ATtiny45__ ) | \
     defined( __AVR_ATtiny85__ ) | \
     defined( __AVR_ATtiny261__ ) | \
     defined( __AVR_ATtiny461__ ) | \
     defined( __AVR_ATtiny861__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny26__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STRT_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATmega165__ ) | \
     defined( __AVR_ATmega325__ ) | \
     defined( __AVR_ATmega3250__ ) | \
     defined( __AVR_ATmega645__ ) | \
     defined( __AVR_ATmega6450__ ) | \
     defined( __AVR_ATmega329__ ) | \
     defined( __AVR_ATmega3290__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATmega169__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif



/********************************************************************************

                        functions implemented as macros

********************************************************************************/

#define SET_USI_TO_SEND_ACK( ) \
{ \
  /* prepare ACK */ \
  USIDR = 0; \
  /* set SDA as output */ \
  DDR_USI |= ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC )| \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* prepare ACK */ \
  USIDR = 0; \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | \
       ( 1 << USIDC ) | \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE( ) \
{ \
  USICR = \
       /* enable Start Condition Interrupt, disable Overflow Interrupt */ \
       ( 1 << USISIE ) | ( 0 << USIOIE ) | \
       /* set USI in Two-wire mode, no USI Counter overflow hold */ \
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
       /* Shift Register Clock Source = External, positive edge */ \
       /* 4-Bit Counter Source = external, both edges */ \
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
       /* no toggle clock-port pin */ \
       ( 0 << USITC ); \
  USISR = \
        /* clear all interrupt flags, except Start Cond */ \
        ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
        ( 1 << USIDC ) | ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_SEND_DATA( ) \
{ \
  /* set SDA as output */ \
  DDR_USI |=  ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    =  \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_READ_DATA( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    = \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | ( 1 << USIDC ) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
}

#define USI_RECEIVE_CALLBACK() \
{ \
    if (usi_onReceiverPtr) \
    { \
        if (usiTwiAmountDataInReceiveBuffer()) \
        { \
            usi_onReceiverPtr(usiTwiAmountDataInReceiveBuffer()); \
        } \
    } \
}

#define ONSTOP_USI_RECEIVE_CALLBACK() \
{ \
    if (USISR & ( 1 << USIPF )) \
    { \
        USI_RECEIVE_CALLBACK(); \
    } \
}


#define USI_REQUEST_CALLBACK() \
{ \
    USI_RECEIVE_CALLBACK(); \
    if(usi_onRequestPtr) usi_onRequestPtr(); \
}

/********************************************************************************

                                   typedef's

********************************************************************************/

typedef enum
{
  USI_SLAVE_CHECK_ADDRESS                = 0x00,
  USI_SLAVE_SEND_DATA                    = 0x01,
  USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
  USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
  USI_SLAVE_REQUEST_DATA_START           = 0x04,
  USI_SLAVE_GET_DATA_AND_SEND_ACK_START  = 0x05,
  USI_SLAVE_REQUEST_DATA_NEXT            = 0x06,
  USI_SLAVE_GET_DATA_AND_SEND_ACK_NEXT   = 0x07
} overflowState_t;



/********************************************************************************

                                local variables

********************************************************************************/

#define TWI_RX_BUFFER_SIZE sizeof(registers_t)
#define TWI_TX_BUFFER_SIZE sizeof(registers_t)


static uint8_t                  slaveAddress;
static volatile overflowState_t overflowState;


uint8_t *twi_rx_buf = (uint8_t *)(&in_regs);
static volatile uint8_t twi_rx_count;
uint8_t *twi_tx_buf = (uint8_t *)(&out_regs);
static volatile uint8_t twi_tx_count;
static volatile uint8_t twi_reg;



/********************************************************************************

                                local functions

********************************************************************************/



// flushes the TWI buffers

static void twi_reset_buffers(void)
{
  twi_rx_count = 0;
  twi_tx_count = 0;
  twi_reg = 0;
} // end flushTwiBuffers



/********************************************************************************

                                public functions

********************************************************************************/



// initialise USI for TWI slave mode

void twi_init(uint8_t ownAddress)
{

  twi_reset_buffers( );

  slaveAddress = ownAddress;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  // Set SCL and SDA as output
  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

  // set SCL high
  PORT_USI |= ( 1 << PORT_USI_SCL );

  // set SDA high
  PORT_USI |= ( 1 << PORT_USI_SDA );

  // Set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  USICR =
       // enable Start Condition Interrupt
       ( 1 << USISIE ) |
       // disable Overflow Interrupt
       ( 0 << USIOIE ) |
       // set USI in Two-wire mode, no USI Counter overflow hold
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       // Shift Register Clock Source = external, positive edge
       // 4-Bit Counter Source = external, both edges
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       // no toggle clock-port pin
       ( 0 << USITC );

  // clear all interrupt flags and reset overflow counter

  USISR = ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );

} // end usiTwiSlaveInit


int8_t twi_has_transmitted(void)
{
    static uint8_t twi_last_tx_count = 0;
    if (twi_last_tx_count != twi_tx_count)
    {
        twi_last_tx_count = twi_tx_count;
        return 1;
    }
    return 0;
} 

int8_t twi_has_received(void)
{
    static uint8_t twi_last_rx_count = 0;
    if (twi_last_rx_count != twi_rx_count)
    {
        twi_last_rx_count = twi_rx_count;
        return 1;
    }
    return 0;
} 

void twi_close(void)
{
    USICR = 0;
    // Set SCL and SDA as input
    DDR_USI &= ~(( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA ));
}

/********************************************************************************

                            USI Start Condition ISR

********************************************************************************/

ISR( USI_START_VECTOR )
{
  // set default starting conditions for new TWI package
  overflowState = USI_SLAVE_CHECK_ADDRESS;

  // set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  // wait for SCL to go low to ensure the Start Condition has completed (the
  // start detector will hold SCL low ) - if a Stop Condition arises then leave
  // the interrupt to prevent waiting forever - don't use USISR to test for Stop
  // Condition as in Application Note AVR312 because the Stop Condition Flag is
  // going to be set from the last TWI sequence
  while (
       // SCL his high
       ( PIN_USI & ( 1 << PIN_USI_SCL ) ) &&
       // and SDA is low
       !( ( PIN_USI & ( 1 << PIN_USI_SDA ) ) )
  );


  if ( !( PIN_USI & ( 1 << PIN_USI_SDA ) ) )
  {

    // a Stop Condition did not occur

    USICR =
         // keep Start Condition Interrupt enabled to detect RESTART
         ( 1 << USISIE ) |
         // enable Overflow Interrupt
         ( 1 << USIOIE ) |
         // set USI in Two-wire mode, hold SCL low on USI Counter overflow
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
         // Shift Register Clock Source = External, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  }
  else
  {
    // a Stop Condition did occur

    USICR =
         // enable Start Condition Interrupt
         ( 1 << USISIE ) |
         // disable Overflow Interrupt
         ( 0 << USIOIE ) |
         // set USI in Two-wire mode, no USI Counter overflow hold
         ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
         // Shift Register Clock Source = external, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  } // end if

  USISR =
       // clear interrupt flags - resetting the Start Condition Flag will
       // release SCL
       ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
       ( 1 << USIPF ) |( 1 << USIDC ) |
       // set USI to sample 8 bits (count 16 external SCL pin toggles)
       ( 0x0 << USICNT0);


} // end ISR( USI_START_VECTOR )



/********************************************************************************

                                USI Overflow ISR

Handles all the communication.

Only disabled when waiting for a new Start Condition.

********************************************************************************/

ISR( USI_OVERFLOW_VECTOR )
{

  switch ( overflowState )
  {

      // Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK,
      // else reset USI
      case USI_SLAVE_CHECK_ADDRESS:
          if ( ( USIDR == 0 ) || ( ( USIDR >> 1 ) == slaveAddress) )
          {
              if ( USIDR & 0x01 )
              {
                  overflowState = USI_SLAVE_SEND_DATA;
              }
              else
              {
                  overflowState = USI_SLAVE_REQUEST_DATA_START;
              } // end if
              SET_USI_TO_SEND_ACK( );
          }
          else
          {
              SET_USI_TO_TWI_START_CONDITION_MODE( );
          }
          break;

          // Master write data mode: check reply and goto USI_SLAVE_SEND_DATA if OK,
          // else reset USI
      case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
          if ( USIDR )
          {
              // if NACK, the master does not want more data
              SET_USI_TO_TWI_START_CONDITION_MODE( );
              return;
          }
          // from here we just drop straight into USI_SLAVE_SEND_DATA if the
          // master sent an ACK

          // copy data from buffer to USIDR and set USI to shift byte
          // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
      case USI_SLAVE_SEND_DATA:
          // Get data from Buffer
          if ( twi_reg < TWI_TX_BUFFER_SIZE )
          {
              USIDR = twi_tx_buf[ twi_reg++ ];
              twi_tx_count++;
          }
          else
          {
              // the buffer is empty
              SET_USI_TO_READ_ACK( ); // This might be neccessary sometimes see http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=805227#805227
              SET_USI_TO_TWI_START_CONDITION_MODE( );
              return;
          } // end if
          overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
          SET_USI_TO_SEND_DATA( );
          break;

          // set USI to sample reply from master
          // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
      case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
          overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
          SET_USI_TO_READ_ACK( );
          break;

          ///////////////////////////////////////////////////////////////////
          // Master read data mode: set USI to sample data from master, next
          // USI_SLAVE_GET_DATA_AND_SEND_ACK
      case USI_SLAVE_REQUEST_DATA_START:
          overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK_START;
          SET_USI_TO_READ_DATA( );
          break;

          // copy data from USIDR and send ACK
          // next USI_SLAVE_REQUEST_DATA
      case USI_SLAVE_GET_DATA_AND_SEND_ACK_START:
          // put data into buffer
          // check buffer size
          twi_reg = USIDR;
          // next USI_SLAVE_REQUEST_DATA
          overflowState = USI_SLAVE_REQUEST_DATA_NEXT;
          SET_USI_TO_SEND_ACK( );
          break;

          ///////////////////////////////////////////////////////////////////
          // Master read data mode: set USI to sample data from master, next
          // USI_SLAVE_GET_DATA_AND_SEND_ACK
      case USI_SLAVE_REQUEST_DATA_NEXT:
          overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK_NEXT;
          SET_USI_TO_READ_DATA( );
          break;

          // copy data from USIDR and send ACK
          // next USI_SLAVE_REQUEST_DATA
      case USI_SLAVE_GET_DATA_AND_SEND_ACK_NEXT:
          // put data into buffer
          // check buffer size
          if ( twi_reg < TWI_RX_BUFFER_SIZE )
          {
              twi_rx_buf[ twi_reg++ ] = USIDR;
              twi_rx_count++;
          } else {
              // overrun
              // drop data
          }
          // next USI_SLAVE_REQUEST_DATA
          overflowState = USI_SLAVE_REQUEST_DATA_NEXT;
          SET_USI_TO_SEND_ACK( );
          break;
  } // end switch

} // end ISR( USI_OVERFLOW_VECTOR )
