
#include <ad7689.h>

AD7689 *adc;
const uint8_t numberChannels = 3;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  adc = new AD7689(10);
  Serial.println("selftest: " + String(adc->selftest() ? "success" : "failed"));
}
uint8_t i = 0;
void loop() {
  uint32_t timestamp;
  if(!i){
    Serial.println("Temp: " + String(adc->acquireTemperature()));
  }
  Serial.print("voltage channel["+ String(i)+"] ");
  Serial.print(adc->acquireChannel(i, &timestamp), DEC);
  Serial.print("    timestamp: "); Serial.println(timestamp, DEC);
  i= (i+1)%8;
  //delay(200);
}
