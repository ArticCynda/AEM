#include "ad7689.h"

ADC7689::ADC7689(uint8_t SSpin, float vref) : AD7689_settings(F_CPU >= MAX_FREQ ? MAX_FREQ : 1000000, MSBFIRST, SPI_MODE0), AD7689_PIN(SSpin){ // set SPI clock to maximum (38 MHz default)
  init(vref);
}

void ADC7689::init(float ref) {
  
  // initialize ADC with default values
  conf.INCC_conf = INCC_UNIPOLAR_REF_GND; // default to single ended reference to ground
  conf.INx_conf = 0; // default to reading channel 0
  conf.BW_conf = 1; // default to full bandwidth
  //conf.REF_conf = INT_REF_25; // default to internal 2.5V reference
  //conf.REF_voltage = 2.5;
  conf.SEQ_conf = SEQ_OFF; // turn off sequencing by default

  if (ref == 2.5) {
    //setConfig(INCC_UNIPOLAR_REF_GND, 0, 0, INT_REF_25, SEQ_OFF); // use internal 2.5V reference
    conf.REF_conf = INT_REF_25;
  } else if (ref == 4.096) {
    //setConfig(INCC_UNIPOLAR_REF_GND, 0, 0, INT_REF_4096, SEQ_OFF); // use internal 4.096V reference
    conf.REF_conf = INT_REF_4096;
  } else {
    //setConfig(INCC_UNIPOLAR_REF_GND, 0, 0, EXT_REF_TEMP_BUF, SEQ_OFF); // use external reference w. buffer
    conf.REF_conf = EXT_REF_TEMP_BUF;
  }
  conf.REF_voltage = ref;

#ifdef DEBUG
  Serial.print("REF: "); Serial.println(conf.REF_conf, HEX);
  Serial.print("REF V: "); Serial.println(conf.REF_voltage, DEC);
#endif
  setConfig();
}


//void setConfig(uint8_t chconf, uint8_t channel, bool bandwidth, uint8_t refsource, uint8_t sequencer) {
void ADC7689::setConfig() {

  // select channel and other config
  ad7689_config = 0;
  ad7689_config |= 1 << CFG;		// update config on chip
  ad7689_config |= (conf.INCC_conf & 0b111) << INCC;	// mode - single ended, differential, ref, etc
  ad7689_config |= (conf.INx_conf & 0b111) << INx;	// channel
  ad7689_config |= conf.BW_conf << BW;		// 1 adds more filtering
  ad7689_config |= (conf.REF_conf & 0b111) << REF; // internal 4.096V reference
  //ad7689_config |= 0B0 << REF;	// use internal 2.5V reference
  //ad7689_config |= 0B110 << REF;	// use external reference (maybe ~3.3V)
  ad7689_config |= (conf.SEQ_conf & 0b11) << SEQ;		// don't auto sequence
  //ad7689_config |= 1 << RB;		// don't read back config value
  ad7689_config |= 0 << RB;		// read back config value

  ad7689_config = ad7689_config << 2;   // convert 14 bits to 16 bits

#ifdef DEBUG
  Serial.println("new configuration:");
  Serial.print("set INCC: "); Serial.println((conf.INCC_conf & 0b111), BIN);
  Serial.print("set INx: "); Serial.println((conf.INx_conf & 0b111), BIN);
  Serial.print("set BW: "); Serial.println(conf.BW_conf, BIN);
  Serial.print("set REF: "); Serial.println((conf.REF_conf & 0b111), BIN);
  Serial.print("set SEQ: "); Serial.println((conf.SEQ_conf & 0b11), BIN);
  Serial.print("config value: "); Serial.println(ad7689_config, HEX);
#endif
  pinMode(AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  SPI.beginTransaction(AD7689_settings);
  digitalWrite(AD7689_PIN, LOW);
  SPI.transfer(ad7689_config >> 8);	// high byte
  SPI.transfer(ad7689_config & 0xFF);	// low byte, 2 bits ignored
  
  digitalWrite(AD7689_PIN, HIGH);
  SPI.endTransaction();

  delayMicroseconds(AD_DELAY);

  // change the readback flag back to 1
  uint16_t change_config = ad7689_config;
  ad7689_config = ad7689_config | 0x4;
  
  SPI.beginTransaction(AD7689_settings);
  digitalWrite(AD7689_PIN, LOW);
  SPI.transfer(ad7689_config >> 8);
  SPI.transfer(ad7689_config & 0xFF);
  digitalWrite(AD7689_PIN, HIGH);
  SPI.endTransaction ();
  
  delayMicroseconds(AD_DELAY);

  SPI.beginTransaction(AD7689_settings);
  digitalWrite(AD7689_PIN, LOW);
  uint16_t retval = SPI.transfer(ad7689_config >> 8) << 8;
  retval |= SPI.transfer(ad7689_config & 0xFF);
  uint16_t retval2 = SPI.transfer(ad7689_config >> 8) << 8;
  retval2 |= SPI.transfer(ad7689_config & 0xFF);
  digitalWrite(AD7689_PIN, HIGH);
  SPI.endTransaction();

  bool changeset = (change_config == retval2);
  
  delayMicroseconds(2); // minumum 1.2µs

  if (changeset) {
    #ifdef DEBUG
      Serial.println("success!");
      Serial.println(retval, HEX);
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("failure");
    #endif
  }
  return changeset;
}

// do conversion and return result
// assume that the channel is already set correctly
// sample & hold and conversion starts immediately

//inline uint16_t read_AD7689() __attribute__((always_inline));

uint16_t ADC7689::read_AD7689 () const{

  // do conversion
  digitalWrite(AD7689_PIN, LOW);         // chip select
  digitalWrite(AD7689_PIN, HIGH);        // chip deselect  (starts conversion)
  delayMicroseconds(AD_DELAY);                      // wait till complete

  // read conversion result
  SPI.beginTransaction (AD7689_settings);
  digitalWrite (AD7689_PIN, LOW);            // chip select
  uint16_t val = SPI.transfer (ad7689_config  >> 8) << 8;   // high byte
  val |= SPI.transfer (ad7689_config);	    // low byte
  digitalWrite (AD7689_PIN, HIGH);          // chip select
  delayMicroseconds(AD_DELAY);                     // wait for second conversion to complete
  SPI.endTransaction ();

  return val;
}

float ADC7689::readVoltage(uint8_t AIN) {
  if (AIN != conf.INx_conf) {
    // reading from another channel, reconfigure the ADC
    conf.INx_conf = AIN;
    setConfig();
  }
  return (read_AD7689() * conf.REF_voltage / 65536);
}

uint16_t ADC7689::shiftTransaction(uint16_t command, bool readback, uint16_t* rb_cmd_ptr) {
  // one time start-up sequence
  if (!init_complete) {
    // give ADC time to start up
    delay(100);

    // synchronize start of conversion
    digitalWrite(AD7689_PIN, LOW);
    delayMicroseconds(1); // miniumum 10 ns
    digitalWrite(AD7689_PIN, HIGH);
    delayMicroseconds(AD_DELAY); // minimum 3.2 µs
    init_complete = true;
  }

  pinMode(AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  // allow time to sample
  delayMicroseconds(AD_DELAY);

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

uint16_t ADC7689::toCommand(AD7689_conf cfg) const{
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
AD7689_conf ADC7689::getDefaultConfig() const {
  AD7689_conf def;
  def.CFG_conf = false;                   // don't update the existing configuration
  def.INCC_conf = INCC_UNIPOLAR_REF_GND;  // use unipolar inputs, with reference to ground
  def.INx_conf = 0;                       // read channel 0
  def.BW_conf = 1;                        // full bandwidth
  def.REF_conf = INT_REF_4096;            // use interal 4.096V reference voltage
  def.SEQ_conf = SEQ_OFF;                 // disable sequencer
  def.RB_conf = false;                    // disable readback

  return def;
}

// returns a value indicating if the ADC is properly connected and responding
bool ADC7689::selftest() {
  // ADC will be tested with its readback function, which reads back a previous command
  // this process takes 3 cycles

  AD7689_conf rb_conf = getDefaultConfig();
  rb_conf.CFG_conf = true;   // overwrite previous configuration
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
