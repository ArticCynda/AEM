
/*
Example of USART in SPI mode on the Atmega328.

Author:   Nick Gammon
Date:     12th April 2012
Version:   1.0

Licence: Released for public use.

Pins: D0 MISO (Rx)
      D1 MOSI (Tx)
      D4 SCK  (clock)
      D5 SS   (slave select)  <-- this can be changed

 Registers of interest:

 UDR0 - data register

 UCSR0A – USART Control and Status Register A
     Receive Complete, Transmit Complete, USART Data Register Empty

 UCSR0B – USART Control and Status Register B
     RX Complete Interrupt Enable, TX Complete Interrupt Enable, Data Register Empty Interrupt Enable ,
     Receiver Enable, Transmitter Enable

 UCSR0C – USART Control and Status Register C
     Mode Select (async, sync, SPI), Data Order, Clock Phase, Clock Polarity

 UBRR0L and UBRR0H - Baud Rate Registers - together are UBRR0 (16 bit)

*/

#include "yspi.h"
#include <ad7689.h>

// sends/receives one byte
uint8_t YMSPI::MSPIMTransfer (uint8_t data){
  // enable slave select
  // digitalWrite (MSPIM_SS, LOW);
  /*
  SET_SS(LOW);

  // wait for transmitter ready
  //while ((UCSR0A & _BV (UDRE0)) == 0)
  while ((UCSR1A & _BV (UDRE1)) == 0)
    {}

  // send byte
  //UDR0 = c;
  UDR1 = c;

  // wait for receiver ready
  //while ((UCSR0A & _BV (RXC0)) == 0)
  while ((UCSR1A & _BV (RXC1)) == 0)
    {}

  // disable slave select
  //digitalWrite (MSPIM_SS, HIGH);
  //PORTD4 = 1;
  SET_SS(HIGH);

  // receive byte, return it
  //return UDR0;
  return UDR1;
  */

  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (_BV(UDRE1))));
  /* Put data into buffer, sends the data */
  UDR1 = data;
  /* Wait for data to be received */
  while( !(UCSR1A & (_BV(RXC1))) );
  /* Get and return received data from buffer */
  return UDR1;

}  // end of MSPIMTransfer

YMSPI::YMSPI(uint8_t usartID) {
  //pinMode (MSPIM_SS, OUTPUT);   // SS
  // slave select pin to output
  SET_SS_OUT;

  switch (usartID){
  case 0:
    break;
  case 1:

    // must be zero before enabling the transmitter
    UBRR1 = 0;

    // set XCKn port pin as output, enables master mode
    SET_SCK_OUT;

    /* Set MSPI mode of operation and SPI data mode 0. */
    UCSR1C = _BV(UMSEL11) | _BV(UMSEL10);
    /* Enable receiver and transmitter. */
    UCSR1B = _BV(RXEN1) | _BV(TXEN1);
    /* Set baud rate. */
    /* IMPORTANT: The Baud Rate must be set after the transmitter is enabled */
    UBRR1 = 0; // maximum speed

    //UCSR0C = _BV (UMSEL00) | _BV (UMSEL01);  // Master SPI mode
    //UCSR1C = _BV (UMSEL10) | _BV (UMSEL11);  // Master SPI mode

    //UCSR0B = _BV (TXEN0) | _BV (RXEN0);  // transmit enable and receive enable
    //UCSR1B = _BV (TXEN1) | _BV (RXEN1);  // transmit enable and receive enable

    // must be done last, see page ??206??
    //UBRR1 = 0;  // 2 Mhz clock rate
    break;
  case 2:
    break;
  case 3:
    break;
  }
}
