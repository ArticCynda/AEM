#include "ad7689.h"

void AD7689::configureSequencer(AD7689_conf sequence) {

  // turn on sequencer if it hasn't been turned on yet, and set it to read temperature too
  sequence.SEQ_conf = SEQ_SCAN_INPUT_TEMP;
  // disable readback
  sequence.RB_conf = false;
  // overwrite existing command
  sequence.CFG_conf = true;

  // convert ADC configuration to command word
  uint16_t command = toCommand(sequence);

  // send command to configure ADC and enable sequencer
  shiftTransaction(command, false, NULL);;

  // skip a frame
  shiftTransaction(0, false, NULL);
}

void AD7689::init(uint8_t SSpin, uint8_t refSource, float ref) {
  SPI.begin();

  AD7689_PIN = SSpin;
  pinMode(SSpin, OUTPUT);

#ifdef DEBUG
  Serial.print("REF: "); Serial.println(conf.REF_conf, HEX);
  Serial.print("REF V: "); Serial.println(conf.REF_voltage, DEC);
#endif

}

// reads voltages from selected channels, always read temperature too
// params:
//   channels: last channel to read (starting at 0, max 7, in differential mode always read even number of channels)
//   mode: unipolar, bipolar or differential
//   data: pointer to a vector holding the data, length depending on channels and mode
//   temp: pointer to a variable holding the temperature
void AD7689::readChannels(uint8_t channels, uint8_t mode, uint16_t* data, uint16_t* temp) {

  uint8_t scans = channels; // unipolar mode default
  if (mode == DIFFERENTIAL_MODE) {
    scans = channels / 2;
    if ((channels % 2) > 0)
      scans++;
  }

  // read as many values as there are ADC channels active
  // when reading differential, only half the number of channels will be read
  for (uint8_t ch = 0; ch < scans; ch++) {
    uint16_t retval = shiftTransaction(0, false, NULL);
    *(data + ch) = retval;
  }

  // capture temperature too
  uint16_t t = shiftTransaction(0, false, NULL);
  *temp = t;
}

/* sends a 16 bit word to the ADC, and simultaneously captures the response
   ADC responses lag 2 frames behind on commands
   if readback is activated, 32 bits will be captured instead of 16
*/

uint16_t AD7689::shiftTransaction(uint16_t command, bool readback, uint16_t* rb_cmd_ptr) {

  // one time start-up sequence
  if (!init_complete) {
    // give ADC time to start up
    delay(100);

    // synchronize start of conversion
    digitalWrite(AD7689_PIN, LOW);
    delayMicroseconds(1); // miniumum 10 ns
    digitalWrite(AD7689_PIN, HIGH);
    delayMicroseconds(4); // minimum 3.2 µs
    init_complete = true;
  }

  //pinMode(AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  // allow time to sample
  delayMicroseconds(4);

  // send config (RAC mode) and acquire data
  SPI.beginTransaction(AD7689_settings);
  digitalWrite(AD7689_PIN, LOW); // activate the ADC

  uint16_t data = SPI.transfer(command >> 8) << 8;  // transmit 8 MSB
  data |= SPI.transfer(command & 0xFF);             // transmit / LSB

  // if a readback is requested, the 16 bit frame is extended with another 16 bits to retrieve the value
  if (readback) {
    // duplicate previous command
    uint16_t readback_value = SPI.transfer(command >> 8) << 8;
    readback_value |= SPI.transfer(command & 0xFF);

    // return readback value to the calling function
    *rb_cmd_ptr = readback_value;
  }

  digitalWrite(AD7689_PIN, HIGH); // release the ADC
  SPI.endTransaction();

  // delay to allow data acquisition for the next cycle
  delayMicroseconds(2); // minumum 1.2µs

  return data;
}

// converts a command structure to a 16 bit word that can be transmitted over SPI
uint16_t AD7689::toCommand(AD7689_conf cfg) const {

  // bit shifts needed for config register values, from datasheet p. 27 table 11:
  #define CFG 13
  #define INCC 10
  #define INx 7
  #define BW  6
  #define REF 3
  #define SEQ 1
  #define RB 0

  // build 14 bit configuration word
  uint16_t command = 0;
  command |= cfg.CFG_conf << CFG;		// update config on chip
  command |= (cfg.INCC_conf & 0b111) << INCC;	// mode - single ended, differential, ref, etc
  command |= (cfg.INx_conf & 0b111) << INx;	// channel
  command |= cfg.BW_conf << BW;		// 1 adds more filtering
  command |= (cfg.REF_conf & 0b111) << REF; // internal 4.096V reference
  command |= (cfg.SEQ_conf & 0b11) << SEQ;		// don't auto sequence
  command |= !(cfg.RB_conf) << RB;		// read back config value

  // convert 14 bits to 16 bits, 2 LSB are don't cares
  command = command << 2;

  return command;
}

// returns an ADC confuration loaded with the default settings, for testing purposes
AD7689_conf AD7689::getDefaultConfig() const {
  AD7689_conf def;
  def.CFG_conf = true;                    // overwrite existing configuration
  def.INCC_conf = INCC_UNIPOLAR_REF_GND;  // use unipolar inputs, with reference to ground
  def.INx_conf = 0;                       // read channel 0
  def.BW_conf = 1;                        // full bandwidth
  def.REF_conf = INT_REF_4096;            // use interal 4.096V reference voltage
  def.SEQ_conf = SEQ_OFF;                 // disable sequencer
  def.RB_conf = false;                    // disable readback

  return def;
}

// returns a value indicating if the ADC is properly connected and responding
bool AD7689::selftest() {
  // ADC will be tested with its readback function, which reads back a previous command
  // this process takes 3 cycles

  AD7689_conf rb_conf = getDefaultConfig();
  rb_conf.RB_conf = true;    // enable readback

  // send readback command
  shiftTransaction(toCommand(rb_conf), false, NULL);

  // skip second frame
  shiftTransaction(toCommand(getDefaultConfig()), false, NULL);

  // capture readback response
  uint16_t readback;
  shiftTransaction(toCommand(getDefaultConfig()), true, &readback);

  // response with initial readback command
  return (readback == toCommand(rb_conf));
}

// preliminary test results:
// raw values range from 4260 at room temperature to over 4400 when heated
// need calibration with ice cubes (= 0°C) and boiling methanol (= 64.7°C) or boiling ether (= 34.6°C)

#define TEMP_REF 4.096  // reference voltage to be used for temperature measurement, either 2.5V or 4.096V
float AD7689::readTemperature() {

  AD7689_conf temp_conf = getDefaultConfig();

  // set to use internal reference voltage
  // this automatically turns on the temperature sensor
  if (TEMP_REF == 2.5)
    temp_conf.REF_conf = INT_REF_25;
  else
    temp_conf.REF_conf = INT_REF_4096;

  // configure MUX for temperature sensor
  temp_conf.INCC_conf = INCC_TEMP;

  digitalWrite(AD7689_PIN, LOW);
  digitalWrite(AD7689_PIN, HIGH);
  delayMicroseconds(4);

  // send the command
  shiftTransaction(toCommand(temp_conf), false, NULL);

  // skip second frame
  shiftTransaction(toCommand(getDefaultConfig()), false, NULL);

  // retrieve temperature reading
  uint16_t t = shiftTransaction(toCommand(getDefaultConfig()), false, NULL);

  Serial.print("temp ADC out: "); Serial.println(t, DEC);

  // calculate temperature from ADC value:
  // output is 283 mV @ 25°C, and sensitivity of 1 mV/°C
  float temp = 25 + ((t * TEMP_REF / 65536)- 0.283) * 0.001;


  return temp;
}
