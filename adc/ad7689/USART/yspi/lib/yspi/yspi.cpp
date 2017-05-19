
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

  UCSR1A |= _BV(TXC1); // clear transmit complete flag

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
  // set SS as output (this has been tested, works)
  SET_SS_OUT;

  switch (usartID){
  case 0:
    break;
  case 1:

    // must be zero before enabling the transmitter
    UBRR1 = 0;

    // set XCKn port pin as output, enables master mode (this has been tested, works)
    SET_SCK_OUT;


    /* Set MSPI mode of operation and SPI data mode 0 (CPOL = 0, CPHA = 0) and MSB first. */
    UCSR1C = _BV(UMSEL11) | _BV(UMSEL10);
    /* Enable receiver and transmitter. */
    UCSR1B = _BV(RXEN1) | _BV(TXEN1);

    Serial.print("UCSR1B: "); Serial.println(UCSR1B, BIN);
    Serial.print("UCSR1C: "); Serial.println(UCSR1C, BIN);

    /* Set baud rate. */
    /* IMPORTANT: The Baud Rate must be set after the transmitter is enabled */
    UBRR1 = 416; // maximum speed

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
