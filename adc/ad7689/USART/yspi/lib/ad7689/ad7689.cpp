// Driver code for the Analog Devices AD7689 16 bit, 8 channel, SPI ADC.
// Will use default SPI port of the MCU as specified by the SPI.h header
// All functionality of the chip is currently supported, some of it is untested however
// Report bugs by opening a ticket on Github:
// This driver MAY also work for the similar chip AD7682, but this hasn't been tested.
// Code released under MIT license blah blah blah, use it however you wish, if it has been useful then let me know about your project at yannick [dot] verbelen [at] inventati [dot] org

#include "ad7689.h"
//#include <SPI.h>

/**
 * [AD7689::configureSequencer Enables the automatic channel sequencer of the ADC and turn on temperature measurements.]
 */
void AD7689::configureSequencer() {
  // load a new configuration with the setting specified by the user
  AD7689_conf sequence = getADCConfig();

  // turn on sequencer if it hasn't been turned on yet, and set it to read temperature too
  sequence.SEQ_conf = SEQ_SCAN_INPUT_TEMP;
  // disable readback
  sequence.RB_conf = false;
  // overwrite existing command
  sequence.CFG_conf = true;

  // convert ADC configuration to command word
  uint16_t command = toCommand(sequence);

  // send command to configure ADC and enable sequencer
  shiftTransaction(command, false, NULL);

  // skip a frame
  shiftTransaction(0, false, NULL);
  lastSeqEndTime = micros();

  // remember that the sequencer is active to prevent unnecessary intializations
  sequencerActive = true;
}

/**
 * [AD7689::setReference Configure the voltage references.]
 * @param refSource    Reference source: REF_INTERNAL, REF_EXTERNAL, REF_GND or REF_COM.
 * @param posRef       Positive reference voltage.
 * @param polarity     Polarity of the signal source: UNIPOLAR_MODE, BIPOLAR_MODE or DIFFERENTIAL_MODE.
 * @param differential Indicates if the inputs are differential.
 */
void AD7689::setReference(uint8_t refSource, float posRef, uint8_t polarity, bool differential) {

    // set input configuration and negative reference
    if (differential) {
      if (polarity == BIPOLAR_MODE) {
        // bipolar differential pairs, INx- referenced to Vref/2
        inputConfig = INCC_BIPOLAR_DIFF;
        negref = posRef / 2;
      }  else {
        // unipolar differential pairs, INx- referenced to GND
        inputConfig = INCC_UNIPOLAR_DIFF; // default differential
        negref = 0;
      }

    } else { // single ended
      if (polarity == BIPOLAR_MODE) {
        // bipolar single ended, INx referenced to COM = Vref/2
        inputConfig = INCC_BIPOLAR_COM;
        negref = posRef / 2;

      } else { // unipolar
        // unipolar, Inx referenced to GND
        inputConfig = INCC_UNIPOLAR_REF_GND;
        negref = 0;
      }
    }

    // set positive reference
    if (refSource == REF_INTERNAL)
    {
      if (posRef == INTERNAL_25) {
        refsrc = INT_REF_25;
        posref = INTERNAL_25;
      }
      else if (posRef == INTERNAL_4096) {
        refsrc = INT_REF_4096;
        posref = INTERNAL_4096;
      }
      else {
        posref = INTERNAL_4096;
        refsrc = INT_REF_4096; // default to 4.096V internal voltage reference
      }
    } else { // external reference
      refsrc = EXT_REF_TEMP_BUF;
      posref = posRef;
    }

}

/**
 * [AD7689::enableFiltering Configure filtering to reduce bandwidth to 25%.]
 * @param onOff True to turn on filter, False for full bandwidth.
 */
void AD7689::enableFiltering(bool onOff) {
  filterConfig = onOff;
}


/**
 * [AD7689::acquireChannel Sample analog input signal along with its time stamp.]
 * @param  channel   The channel to sample, between 1 and 8.
 * @param  timeStamp A pointer to a variable in which the time stamp should be stored.
 * @return           Measured voltage.
 */
float AD7689::acquireChannel(uint8_t channel, uint32_t* timeStamp) {
  if (micros() > (timeStamps[channel] + framePeriod * (TOTAL_CHANNELS - 1))) { // sequence outdated, acquire a new one
    uint8_t cycles = 1;
    if (channel < 2) cycles++; // double sequence to update first 2 channels

    // run 1 or 2 sequences depending on the outdated channel
    for (uint8_t cycle = 0; cycle < cycles; cycle++)
      readChannels(inputCount, ((inputConfig == INCC_BIPOLAR_DIFF) || (inputConfig == INCC_UNIPOLAR_DIFF)), samples, &curTemp);
  }

  *timeStamp = timeStamps[channel];

  return samples[channel];
  //return calculateVoltage(samples[channel]);
}

/**
 * [AD7689::calculateVoltage Calculate an absolute or relative voltage based on raw ADC input reading and specified voltage reference(s).]
 * @param  sample The sample to convert. A positive integer between 0 and 65535.
 * @return        Calculated voltage.
 */
float AD7689::calculateVoltage(uint16_t sample) const {
  return (sample * (posref - negref) / TOTAL_STEPS);
}

/**
 * [AD7689::calculateTemp Calculate the ADC temperature based on raw ADC input readin, using internal voltage reference.]
 * @param  temp The sample to convert. A positive integer between 0 and 65535.
 * @return      Temperature in °C.
 */
float AD7689::calculateTemp(uint16_t temp) const {
  // calculate temperature from ADC value:
  // output is 283 mV @ 25°C, and sensitivity of 1 mV/°C
  // calibration is still necessary, so until done properly, this function returns the raw ADC value instead of temperature

  // preliminary test results:
  // raw values range from 4260 at room temperature to over 4400 when heated
  // need calibration with ice cubes (= 0°C) and boiling methanol (= 64.7°C) or boiling ether (= 34.6°C)
  //return BASE_TEMP + ((temp * posref / TOTAL_STEPS)- TEMP_BASE_VOLTAGE) * TEMP_RICO;
  return temp;
}

/**
 * [AD7689::acquireTemperature Measure temperature.]
 * @return Temperature in °C.
 */
float AD7689::acquireTemperature() {
  if (sequencerActive) {
    // when the sequencer is active, check the time stamp of the last temperature sample and take a new measurement if outdated
    if (micros() > (tempTime + framePeriod * (TOTAL_CHANNELS - 1)))  // temperature outdated, acquire a new one
      readChannels(inputCount, ((inputConfig == INCC_BIPOLAR_DIFF) || (inputConfig == INCC_UNIPOLAR_DIFF)), &samples[0], &curTemp);

    return calculateTemp(curTemp);
  } else {
    // sequencer isn't active yet, fetch tempreature directly
    return readTemperature();
  }
}

// constructor, intialize SPI and set SS pin
// set up the speed, mode and endianness of each device
// MODE0: SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
// use CPHA = CPOL = 0
// SPI runs at max CPU clock as long as it is below 38 MHz

/**
 * [AD7689::constructor Create an instance of an AD7689 ADC.]
 * @param SSpin           The microcontroller pin to which the Slave Select of the ADC is connected. Must be a digital I/O pin.
 * @param numberChannels  The highest channel in use for the application, a value between 1 and 8.
 * @return                Instance of the ADC.
 */
/*AD7689::AD7689(uint8_t SSpin, uint8_t numberChannels) : AD7689_settings (F_CPU >= MAX_FREQ ? MAX_FREQ : F_CPU, MSBFIRST, SPI_MODE0) ,
                                                        SS(SSpin),
                                                        inputCount(numberChannels)  {
*/
AD7689::AD7689(uint8_t numberChannels) : inputCount(numberChannels)  {
// initialize SPI transceiver
  // if it's already active, then this doesn't do anything
  //SPI.begin();

  ymspi = new YMSPI(1);

  //pinMode(SS, OUTPUT);    // configure slave select pin as output (not controlled by SPI transceiver)
  //digitalWrite(SS, HIGH); // slave select active low

  // set default configuration options
  inputConfig = INCC_UNIPOLAR_REF_GND;  // default to unipolar mode with negative reference to ground
  refConfig = INT_REF_4096;             // internal 4.096V reference
  filterConfig = false;                 // full bandwidth

  // measure how long it takes to complete a 16-bit r/w cycle using current F_CPU for accurate sample timing
  cycleTimingBenchmark();

  // reset sample time stamps and force an update sequence at the next read command
  initSampleTiming();
  sequencerActive = false; // sequencer disabled by default

  // set reference source and voltage to the most commonly used values
  setReference(REF_INTERNAL, INTERNAL_4096, UNIPOLAR_MODE, false);
}

/**
 * [AD7689::cycleTimingBenchmark Measures the time required to transceive a complete 16 bit frame, using the current CPU clock speed.
 * This is required to generate accurate time stamps, if desired.
 * Should be called once when starting the ADC, or whenever the clock frequency is changed (i.e. dynamic clock switching).]
 */
void AD7689::cycleTimingBenchmark() {

  uint32_t startTime = micros(); // record current CPU time
  uint16_t data;
  // make 10 transactions, then average the duration
  // dummy variable 'data' is assigned to emulate a realistic operation with the retrieved data
  for (uint8_t trans = 0; trans < 10; trans++)
    data += shiftTransaction(toCommand(getADCConfig(false)), false, NULL); // default configuration, no readback

  framePeriod = (micros() - startTime) / 10;
  lastSeqEndTime = startTime;
}

/**
 * [AD7689::readChannels Reads voltages as raw 16 bit ADC samples from selected channels. Temperature also read.]
 * @param channels Last channel to read, starting at 0 to max. 7, in differential mode always read even number of channels.
 * @param mode     Input signal configuration mode: UNIPOLAR_MODE, BIPOLAR_MODE or DIFFERENTIAL_MODE.
 * @param data     Pointer to a vector holding the data, length depending on channels and mode.
 * @param temp     Pointer to a variable holding the temperature.
 */
void AD7689::readChannels(uint8_t channels, uint8_t mode, uint16_t data[], uint16_t* temp) {

  // if the sequencer insn't active yet, enable it
  // occurs after self testing or at start-up
  if (!sequencerActive)
    configureSequencer();

  uint8_t scans = channels; // unipolar mode default
  if (mode == DIFFERENTIAL_MODE) {
    scans = channels / 2;
    if ((channels % 2) > 0)
      scans++;
  }

  // read as many values as there are ADC channels active
  // when reading differential, only half the number of channels will be read
  for (uint8_t ch = 0; ch < scans; ch++) {
    data[ch] = shiftTransaction(0, false, NULL);
    if (ch < 2) { // calculate time stamp based on ending of previous sequence for first 2 frames
      timeStamps[ch] = lastSeqEndTime - (1 - ch) * framePeriod;
    } else {
      timeStamps[ch] = micros() - framePeriod * 2; // sequenceTime in µs, 2 frames lag
    }
  }

  // capture temperature too
  *temp = shiftTransaction(0, false, NULL);
  tempTime = micros() - framePeriod * 2;
  lastSeqEndTime = micros();
}

/**
 * [AD7689::initSampleTiming Reset time stamps for all samples and force an update sequence at the start of the next read command.]
 * @return The current MCU core time.
 */
uint32_t AD7689::initSampleTiming() {
  uint32_t curTime = micros(); // retrieve microcontroller run time in microseconds

  // set time for all samples to current time to force an update sequence
  for (uint8_t i = 0; i < TOTAL_CHANNELS; i++)
    timeStamps[i] = curTime;

  tempTime = curTime;
  return curTime;
}

/* sends a 16 bit word to the ADC, and simultaneously captures the response
   ADC responses lag 2 frames behind on commands
   if readback is activated, 32 bits will be captured instead of 16
*/
/**
 * [AD7689::shiftTransaction Sends a 16 bit word to the ADC, and simultaneously captures the response.
 * ADC responses lag 2 frames behind on commands.
 * If readback is active, 5332 bits will be captured instead of 16.]
 * @param  command    The 16 bit command to send to the ADC.
 * @param  readback   True if readback is desired, otherwise False.
 * @param  rb_cmd_ptr A pointer to a variable to store the readback response.
 * @return            A 16 bit word received from the ADC, as response to the command from 2 frames ago.
 */
uint16_t AD7689::shiftTransaction(uint16_t command, bool readback, uint16_t* rb_cmd_ptr) {

  // one time start-up sequence
  if (!init_complete) {
    // give ADC time to start up
    delay(STARTUP_DELAY);

    // synchronize start of conversion
    //digitalWrite(SS, LOW);
    SET_SS(LOW);
    delayMicroseconds(TACQ); // miniumum 10 ns
    //digitalWrite(SS, HIGH);
    SET_SS(HIGH);
    delayMicroseconds(TCONV); // minimum 3.2 µs
    init_complete = true;
  }

  // allow time to sample
  delayMicroseconds(TCONV);

  // send config (RAC mode) and acquire data
  //SPI.beginTransaction(AD7689_settings);
  //digitalWrite(SS, LOW); // activate the ADC
  SET_SS(LOW);

  //uint16_t data = (SPI.transfer(command >> 8) << 8) | SPI.transfer(command & 0xFF);
  uint16_t data = (ymspi->MSPIMTransfer(command >> 8) << 8) | ymspi->MSPIMTransfer(command & 0xFF);

  // if a readback is requested, the 16 bit frame is extended with another 16 bits to retrieve the value
  if (rb_cmd_ptr && readback) {
    // duplicate previous command
    *rb_cmd_ptr = (ymspi->MSPIMTransfer(command >> 8) << 8) | ymspi->MSPIMTransfer(command & 0xFF);
  }

  //digitalWrite(SS, HIGH); // release the ADC
  SET_SS(HIGH);

  //SPI.endTransaction();

  // delay to allow data acquisition for the next cycle
  delayMicroseconds(TACQ); // minumum 1.2µs

  return data;
}

// converts a command structure to a 16 bit word that can be transmitted over SPI
/**
 * [AD7689::toCommand Converts a command structure to a 16 bit word that can be transmitted over SPI]
 * @param  cfg A configuration set of type AD7689_conf holding the configuration settings.
 * @return     A 16 bit configuration command that can be sent to the ADC with shiftTransaction.
 */
uint16_t AD7689::toCommand(AD7689_conf cfg) const {

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

// assemble user settings into a configuration for the ADC, or return a default configuration
/**
 * [AD7689::getADCConfig Assemble user settings into a configuration for the ADC, or return a default configuration.]
 * @param  default_config True if the default configuration should be returned, False if user settings are to be used.
 * @return                Configuration set for the ADC.
 */
AD7689_conf AD7689::getADCConfig(bool default_config) {
  AD7689_conf def;

  def.CFG_conf   = true;                    // overwrite existing configuration
  def.INCC_conf  = INCC_UNIPOLAR_REF_GND;   // default unipolar inputs, with reference to ground
  def.INx_conf   = TOTAL_CHANNELS;          // read all channels
  def.BW_conf    = true;                    // full bandwidth
  def.REF_conf   = INT_REF_4096;            // use interal 4.096V reference voltage
  def.SEQ_conf   = SEQ_OFF;                 // disable sequencer
  def.RB_conf    = false;                   // disable readback

  if (!default_config) { // default settings preloaded
    def.INCC_conf  = inputConfig;
    def.INx_conf   = (inputCount - 1);
    def.BW_conf    = !filterConfig;
    def.REF_conf   = refConfig;
  }

  // sequencer disabled, remember to restart it when taking measurements
  sequencerActive = false;

  return def;
}

// returns a value indicating if the ADC is properly connected and responding
/**
 * [AD7689::selftest Verifies that the ADC is properly connected and operational]
 * @return True if the ADC works properly, False if errors were encountered. Check SPI connections if selftest fails repeatedly.
 */
bool AD7689::selftest() {
  // ADC will be tested with its readback function, which reads back a previous command
  // this process takes 3 cycles

  AD7689_conf rb_conf = getADCConfig(true);
  rb_conf.RB_conf = true;    // enable readback

  // send readback command
  shiftTransaction(toCommand(rb_conf), false, NULL);

  // skip second frame
  shiftTransaction(toCommand(getADCConfig(false)), false, NULL);

  // capture readback response
  uint16_t readback;
  shiftTransaction(toCommand(getADCConfig(false)), true, &readback);

  // response with initial readback command
  return (readback == toCommand(rb_conf));
}

/**
 * [AD7689::readTemperature Reads the temperature of the ADC.
 * This function is meant to be called if the ADC is *only* used as a temperature sensor.
 * Whenever actual ADC values are read, temperature is read along with it, and returned directly.
 * This function disables the sequencer.]
 * @return Internal temperature of the ADC in °C.
 */
float AD7689::readTemperature() {

  AD7689_conf temp_conf = getADCConfig(false);

  // set to use internal reference voltage
  // this automatically turns on the temperature sensor
  temp_conf.REF_conf = TEMP_REF == INTERNAL_25 ? INT_REF_25 : INT_REF_4096;

  // configure MUX for temperature sensor
  temp_conf.INCC_conf = INCC_TEMP;

  digitalWrite(SS, LOW);
  digitalWrite(SS, HIGH);
  delayMicroseconds(TCONV);

  // send the command
  shiftTransaction(toCommand(temp_conf), false, NULL);
  sequencerActive = false;

  // skip second frame
  shiftTransaction(toCommand(getADCConfig(false)), false, NULL);

  // retrieve temperature reading
  uint16_t t = shiftTransaction(toCommand(getADCConfig(false)), false, NULL);

  // calculate temperature from ADC value:
  // output is 283 mV @ 25°C, and sensitivity of 1 mV/°C
  return calculateTemp(t);
}
