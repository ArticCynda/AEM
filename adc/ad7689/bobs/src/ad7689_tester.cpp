#include <Arduino.h>
#include "ad7689.h"

ADC7689 *adc;

void setup (){
  // initialize SPI:
  SPI.begin ();
  delay(1000); // give ADC time to boot
  //set_AD7689(7); // set channel
  //setConfig(INCC_UNIPOLAR_REF_GND, 6, 0, INT_REF_25, SEQ_OFF);


  Serial.begin(115200);
  while(!Serial);

  Serial.print("\nCPU speed: "); Serial.println(F_CPU, DEC);
  adc = new ADC7689(10, 4.096);
}

void loop (){
  if (adc->selftest())
    Serial.println("selftest succeeded!");
  else
    Serial.println("selftest failed!");
  delay(500);
}
