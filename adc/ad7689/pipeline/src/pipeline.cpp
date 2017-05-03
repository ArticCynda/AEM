/* SUMMARY
   Speed up DAQ by pipelining commands and responses
*/

#include <SPI.h>		// include the new SPI library:

// input configuration: bipolar/unipolar, single ended or differential
#define INCC_BIPOLAR_DIFF     0b000 // 00X
#define INCC_BIPOLAR_COM      0b010
#define INCC_TEMP             0b011
#define INCC_UNIPOLAR_DIFF    0b100 // 10X
#define INCC_UNIPOLAR_REF_COM 0b110
#define INCC_UNIPOLAR_REF_GND 0b111

// reference voltage (note: decoupling caps required on REF/REFIN when using INT_REF!)
#define INT_REF_25            0b000
#define INT_REF_4096          0b001
#define EXT_REF_TEMP_ON       0b010
#define EXT_REF_TEMP_BUF      0b011
#define EXT_REF_TEMP_OFF      0b110
#define EXT_REF_TEMP_OFF_BUF  0b111

// sequencer configuration (default: no sequencer)
#define SEQ_OFF               0b00
#define SEQ_UPDATE            0b01
#define SEQ_SCAN_INPUT_TEMP   0b10
#define SEQ_SCAN_INPUT        0b11

#define MAX_FREQ              38000000 // 26 ns period @ VDD 5V and VIO 3.3 - 5V

//#define DEBUG

struct AD7689_conf {
  bool    CFG_conf;
  uint8_t INCC_conf;
  uint8_t INx_conf;
  uint8_t BW_conf;
  uint8_t REF_conf;
  uint8_t SEQ_conf;
  float   REF_voltage;
  bool    RB_conf;
};

AD7689_conf conf;

//void set_AD7689 (uint8_t);
//uint16_t read_AD7689 (void);
//float readVoltage(uint8_t AIN);
void setConfig(void);
void init(uint8_t SSpin, float vref);
bool selftest(void);
AD7689_conf getDefaultConfig(void);
uint16_t toCommand(AD7689_conf);
float readTemperature(void);

static bool init_complete = false;

void setup ()
{
  // initialize SPI:
  SPI.begin ();
  delay(1000); // give ADC time to boot
  //set_AD7689(7); // set channel
  //setConfig(INCC_UNIPOLAR_REF_GND, 6, 0, INT_REF_25, SEQ_OFF);


  Serial.begin(115200);
  while(!Serial);

  //Serial.print("\nCPU speed: "); Serial.println(F_CPU, DEC);
  //init(10, 4.096);
}

void loop ()
{
//  delayMicroseconds(200);

if (selftest())
  Serial.println("selftest succeeded!");
else
  Serial.println("selftest failed!");

  setConfig();


  //Serial.println(read_AD7689());            // read value with precise capture time
  //Serial.print("read channel 0: ");
  //Serial.println(readVoltage(0));
//  Serial.print("read channel 2: ");
  //Serial.println(readVoltage(2));
  //Serial.print("temp: ");
  //Serial.println(readTemperature());


  //delay(5);

} // loop()


//*************************************************

// AD7689 16 bit SPI A/D converter interface
// Supports highly accurate sample time
static uint8_t AD7689_PIN = 10;		// chip select pin to use (10 is standard)
#define AD_DELAY   4    // delay from datasheet, default 6 µs

// set up the speed, mode and endianness of each device
// MODE0: SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
// use CPHA = CPOL = 0
// two dummy conversions are required on startup
#if (F_CPU >= MAX_FREQ)
  SPISettings AD7689_settings (MAX_FREQ, MSBFIRST, SPI_MODE0); // set SPI clock to maximum (38 MHz default)
#else
  //SPISettings AD7689_settings (F_CPU, MSBFIRST, SPI_MODE0); // set SPI clock to CPU clock
  SPISettings AD7689_settings (1000000, MSBFIRST, SPI_MODE0); // set SPI clock to CPU clock
#endif


// last device configuration
static uint16_t ad7689_config = 0;

void init(uint8_t SSpin, float ref) {
  AD7689_PIN = SSpin;

/*
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
  */

#ifdef DEBUG
  Serial.print("REF: "); Serial.println(conf.REF_conf, HEX);
  Serial.print("REF V: "); Serial.println(conf.REF_voltage, DEC);
#endif


  //setConfig();
}

/*
void set_AD7689 (uint8_t channel) {

  // bit shifts needed for config register values, from datasheet p. 27 table 11:
  #define CFG 13
  #define INCC 10
  #define INx 7
  #define BW  6
  #define REF 3
  #define SEQ 1
  #define RB 0

  // mask 8 bit input channel to 4 bit input channel
  channel = channel & 0x0F;

  // select channel and other config
  ad7689_config = 0;
  ad7689_config |= 1 << CFG;		// update config on chip
  ad7689_config |= 0B111 << INCC;	// mode - single ended, differential, ref, etc
  ad7689_config |= channel << INx;	// channel
  ad7689_config |= 0 << BW;		// 1 adds more filtering
  ad7689_config |= 0b001 << REF; // internal 4.096V reference
  //ad7689_config |= 0B0 << REF;	// use internal 2.5V reference
  //ad7689_config |= 0B110 << REF;	// use external reference (maybe ~3.3V)
  ad7689_config |= 0 << SEQ;		// don't auto sequence
  ad7689_config |= 0 << RB;		// don't read back config value

  ad7689_config = ad7689_config << 2;   // convert 14 bits to 16 bits

  pinMode (AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  SPI.beginTransaction (AD7689_settings);

  // send config (RAC mode)
  digitalWrite (AD7689_PIN, LOW);
  SPI.transfer (ad7689_config >> 8);	// high byte
  SPI.transfer (ad7689_config & 0xFF);	// low byte, 2 bits ignored
  digitalWrite (AD7689_PIN, HIGH);
  delayMicroseconds(AD_DELAY);

  // dummy
  digitalWrite (AD7689_PIN, LOW);
  SPI.transfer (ad7689_config >> 8);	// high byte
  SPI.transfer (ad7689_config & 0xFF);	// low byte, 2 bits ignored
  digitalWrite (AD7689_PIN, HIGH);
  delayMicroseconds(AD_DELAY);

  SPI.endTransaction ();

}
*/

//void setConfig(uint8_t chconf, uint8_t channel, bool bandwidth, uint8_t refsource, uint8_t sequencer) {

void setConfig() {

  // bit shifts needed for config register values, from datasheet p. 27 table 11:
  #define CFG 13
  #define INCC 10
  #define INx 7
  #define BW  6
  #define REF 3
  #define SEQ 1
  #define RB 0

  // debug
  //conf.REF_conf = INT_REF_4096;
  //conf.INx_conf = 0;

  conf = getDefaultConfig();
  conf.INCC_conf = INCC_UNIPOLAR_REF_GND;
  conf.INx_conf = 7; // read all channels from IN0 up to IN7
  conf.SEQ_conf = SEQ_SCAN_INPUT; // scan all inputs sequentially
  conf.RB_conf = false; // don't read back

/*
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
  */
  ad7689_config = toCommand(conf);

#ifdef DEBUG
  Serial.println("new configuration:");
  Serial.print("set INCC: "); Serial.println((conf.INCC_conf & 0b111), BIN);
  Serial.print("set INx: "); Serial.println((conf.INx_conf & 0b111), BIN);
  Serial.print("set BW: "); Serial.println(conf.BW_conf, BIN);
  Serial.print("set REF: "); Serial.println((conf.REF_conf & 0b111), BIN);
  Serial.print("set SEQ: "); Serial.println((conf.SEQ_conf & 0b11), BIN);
  Serial.print("config value: "); Serial.println(ad7689_config, HEX);
#endif

  //Serial.print("\noriginal:      "); Serial.println(ad7689_config, BIN);

  // DEBUG
  //ad7689_config = 0b1111011100000000;
  //Serial.print("modified:      "); Serial.println(ad7689_config, BIN);


  pinMode(AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  SPI.beginTransaction(AD7689_settings);

  // send config (RAC mode)
  // send twice for dummy conversion
  //for (int i = 0; i < 2; i++) {

    // send sequencer commmand
    digitalWrite(AD7689_PIN, LOW);
    SPI.transfer(ad7689_config >> 8);	// high byte
    SPI.transfer(ad7689_config & 0xFF);	// low byte, 2 bits ignored

    digitalWrite(AD7689_PIN, HIGH);
    SPI.endTransaction();
    //digitalWrite(AD7689_PIN, LOW);
    //delayMicroseconds(1);

    digitalWrite(MOSI, LOW); // keep MOSI low

    delayMicroseconds(4);


    // change the readback flag back to 1
    //uint16_t change_config = ad7689_config;
    //ad7689_config = ad7689_config | 0x4;

    // skip a frame
    SPI.beginTransaction(AD7689_settings);
    digitalWrite(AD7689_PIN, LOW);
    SPI.transfer(ad7689_config >> 8);
    SPI.transfer(ad7689_config & 0xFF);
    digitalWrite(AD7689_PIN, HIGH);
    SPI.endTransaction ();
    //delayMicroseconds(AD_DELAY);
    //}

  delayMicroseconds(4);

  uint16_t channels[8];
  for (uint8_t i = 0; i < (sizeof(channels) / sizeof(channels[0])); i++)
    channels[i] = 0;

  for (uint8_t ch = 0; ch < 8; ch++) {



  SPI.beginTransaction(AD7689_settings);
  digitalWrite(AD7689_PIN, LOW);
  uint16_t retval = SPI.transfer(ad7689_config >> 8) << 8;
  retval |= SPI.transfer(ad7689_config & 0xFF);
  channels[ch] = retval;
  digitalWrite(AD7689_PIN, HIGH);
  SPI.endTransaction();

  //bool changeset = (change_config == retval2);


      //Serial.print("disabled:      "); Serial.println(ad7689_config, BIN);
      delayMicroseconds(2); // minumum 1.2µs

  //Serial.print("return config: "); Serial.println(retval, BIN);
  }

  for (uint8_t ch = 0; ch < 8; ch++) {
    Serial.print("Channel "); Serial.print(ch, DEC); Serial.print(": "); Serial.println(channels[ch], DEC);
  }

/*
  if (changeset) {
    Serial.println("success!");
    Serial.println(retval, HEX);
    return true;
  } else {
    Serial.println("failure");
    return false;
  }
*/

}

// do conversion and return result
// assume that the channel is already set correctly
// sample & hold and conversion starts immediately

//inline uint16_t read_AD7689() __attribute__((always_inline));

/*
uint16_t read_AD7689 ()
{

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

  //Serial.print("read: "); Serial.println(val, HEX);
  return val;

  //return 1;
}
*/

/*
float readVoltage(uint8_t AIN) {
  if (AIN != conf.INx_conf) {
    // reading from another channel, reconfigure the ADC
    conf.INx_conf = AIN;
    setConfig();
  }
  return (read_AD7689() * conf.REF_voltage / 65536);
}
*/

/* sends a 16 bit word to the ADC, and simultaneously captures the response
   ADC responses lag 2 frames behind on commands
   if readback is activated, 32 bits will be captured instead of 16
*/
uint16_t shiftTransaction(uint16_t command, bool readback, uint16_t* rb_cmd_ptr) {

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

  pinMode(AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

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
uint16_t toCommand(AD7689_conf cfg) {

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
AD7689_conf getDefaultConfig() {
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
bool selftest() {
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
/*
#define TEMP_REF 4.096  // reference voltage to be used for temperature measurement, either 2.5V or 4.096V
float readTemperature() {

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
*/
