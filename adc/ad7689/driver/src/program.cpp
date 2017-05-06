
#include "ad7689.h"

AD7689 *adc;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  adc = new AD7689(10);
  adc->setReference(REF_INTERNAL, INTERNAL_4096, UNIPOLAR_MODE, false);
  adc->setInputs(8);




}

void loop() {
  uint32_t timestamp;
  float voltage = adc->acquireChannel(0, &timestamp);

  Serial.print("voltage: "); Serial.println(voltage, DEC);
  delay(200);

}
