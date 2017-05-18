
// this code tests the operation of the AD7689 on the Atmega2560.
// for testing purposes only, not for deployment

#include <ad7689.h>

AD7689 *adc;
//const uint8_t numberChannels = 8;
uint8_t led_state = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //DDD5 = 0; // setting XCK1 as OUTPUT


  pinMode(36, OUTPUT);
  adc = new AD7689(8);
  adc->setReference(REF_EXTERNAL, 2.048, UNIPOLAR_MODE, false);
  Serial.println("selftest: " + String(adc->selftest() ? "success" : "failed"));
}

uint8_t i = 0;
void loop() {
  uint32_t timestamp = 0;
  if(!i){
    Serial.println("Temp: " + String(adc->acquireTemperature()));
  }
  Serial.print("voltage channel["+ String(i)+"] ");
  Serial.print(adc->acquireChannel(i, &timestamp), DEC);
  Serial.print("    timestamp: "); Serial.println(timestamp, DEC);
  i= (i+1)%8;

  digitalWrite(36, led_state);
  led_state = !led_state;
  delay(200);
}
