
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

// sends/receives one byte
uint8_t YMSPI::MSPIMTransfer (uint8_t c){
  // enable slave select
  // digitalWrite (MSPIM_SS, LOW);
  PORTD &= ~_BV(PORTD4);

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
  PORTD = PORTD | _BV(PORTD4);

  // receive byte, return it
  //return UDR0;
  return UDR1;
}  // end of MSPIMTransfer

/*
// select slave, write a string, wait for transfer to complete, deselect slave
void spiWriteString (const char * str)
  {
  if (!str) return;  // Sanity Clause

  char c;

  // enable slave select
  digitalWrite (MSPIM_SS, LOW);

  // send the string
  while (c = *str++)
    MSPIMTransfer (c);

  // wait for all transmissions to finish
  while ((UCSR0A & _BV (TXC0)) == 0)
    {}

  // disable slave select
  digitalWrite (MSPIM_SS, HIGH);
  }  // end of spiWriteString
*/


YMSPI::YMSPI(uint8_t usartID) {
  //pinMode (MSPIM_SS, OUTPUT);   // SS
  // slave select pin to output
  //DDRD = (DDRD | _BV(DDD4)) ^ _BV(DDD4);
  DDRD |= _BV(DDD4);

  switch (usartID){
  case 0:
    break;
  case 1:
    //MSPIM_SCK = UART_SCK_0

    // must be zero before enabling the transmitter
    //UBRR0 = 0;
    UBRR1 = 0;

    //UCSR0A = _BV (TXC0);  // any old transmit now complete
    UCSR1A = _BV (TXC1);  // any old transmit now complete

    //pinMode (MSPIM_SCK, OUTPUT);   // set XCK pin as output to enable master mode
    //DDD5 = 0; // set CLK pin mode to OUTPUT mode
    //DDRD = (DDRD | _BV(DDD5)) ^ _BV(DDD5);
    DDRD |= _BV(DDD5);

    //UCSR0C = _BV (UMSEL00) | _BV (UMSEL01);  // Master SPI mode
    UCSR1C = _BV (UMSEL10) | _BV (UMSEL11);  // Master SPI mode

    //UCSR0B = _BV (TXEN0) | _BV (RXEN0);  // transmit enable and receive enable
    UCSR1B = _BV (TXEN1) | _BV (RXEN1);  // transmit enable and receive enable

    // must be done last, see page ??206??
    UBRR1 = 3;  // 2 Mhz clock rate
    break;
  case 2:
    break;
  case 3:
    break;
  }
}
