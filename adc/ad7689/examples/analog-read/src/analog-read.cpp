// this example illustrates how all ADC channels can be read with default settings:
// unipolar configuration, 8 channels, 4.096V internal positive voltage reference, negative referenced to ground

#include "../../../driver/src/ad7689.h"
#include <SPI.h>

AD7689 *ADC;
const uint8_t AD7689_SS_pin = 10;
uint8_t ch_cnt = 0; // channel counter

void setup() {
  // first initialize the serial debugger
  Serial.begin(115200);
  while(!Serial);

  // AD7689 connected through SPI with SS specified in constructor
  // use default settings (8 channels, unipolar, referenced to 4.096V internal bandga)
  ADC = new AD7689(AD7689_SS_pin);
  if (ADC->selftest())
  {
    Serial.println("AD7689 connected and ready");
  } else {
    Serial.println("Error: couldn't connect to AD7689. Check wiring.");
    while (1);
  }
}

void loop() {
  Serial.print("AD7689 voltage input "+ String(ch_cnt)+" :");
  Serial.println(ADC->acquireChannel(ch_cnt, NULL), DEC);
  ch_cnt = (ch_cnt + 1) % 8;
  delay(250);
}
