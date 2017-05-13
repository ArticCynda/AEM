// this example demonstrates how the AD7689 can be used as a temperature sensor

#include "../../../driver/src/ad7689.h"

AD7689 *ADC;
const uint8_t AD7689_SS_pin  10;

void setup() {
  // first initialize the serial debugger
  Serial.begin(115200);
  while(!Serial);

  // AD7689 connected through SPI with SS specified in constructor
  ADC = new AD7689(AD7689_SS_pin);
  if (ADC->selftest())
  {
    Serial.println("AD7689 connected and ready");
  } else {
    Serial.println("Error: couldn't connect to AD7689. Check wiring.")
    while (1);
  }
}

void loop() {
  Serial.println("AD7689 temperature: " + String(ADC->acquireTemperature()), DEC);
  delay(200);
}
