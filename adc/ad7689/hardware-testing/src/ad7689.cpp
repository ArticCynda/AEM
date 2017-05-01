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
  uint8_t INCC_conf;
  uint8_t INx_conf;
  uint8_t BW_conf;
  uint8_t REF_conf;
  uint8_t SEQ_conf;
  float   REF_voltage;
};

AD7689_conf conf;

//void set_AD7689 (uint8_t);
uint16_t read_AD7689 ();
float readVoltage(uint8_t AIN);
void setConfig();
void init(uint8_t SSpin, float vref);

void setup ()
{
  // initialize SPI:
  SPI.begin ();
  delay(1000); // give ADC time to boot
  //set_AD7689(7); // set channel
  //setConfig(INCC_UNIPOLAR_REF_GND, 6, 0, INT_REF_25, SEQ_OFF);


  Serial.begin(115200);
  while(!Serial);

  Serial.print("\nCPU speed: "); Serial.println(F_CPU, DEC);
  init(10, 4.096);
}

void loop ()
{
//  delayMicroseconds(200);
  //Serial.println(read_AD7689());            // read value with precise capture time
  Serial.print("read channel 0: ");
  Serial.println(readVoltage(0));
  Serial.print("read channel 2: ");
  Serial.println(readVoltage(2));
  delay(50);

} // loop()


//*************************************************

// AD7689 16 bit SPI A/D converter interface
// Supports highly accurate sample time
static uint8_t AD7689_PIN = 10;		// chip select pin to use (10 is standard)
#define AD_DELAY   6    // delay from datasheet, default 6 µs

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
  conf.INx_conf = 1;

/*
  // select channel and other config
  ad7689_config = 0;
  ad7689_config |= 1 << CFG;		// update config on chip
  ad7689_config |= (chconf & 0b111) << INCC;	// mode - single ended, differential, ref, etc
  ad7689_config |= (channel & 0b111) << INx;	// channel
  ad7689_config |= bandwidth << BW;		// 1 adds more filtering
  ad7689_config |= (refsource & 0b111) << REF; // internal 4.096V reference
  //ad7689_config |= 0B0 << REF;	// use internal 2.5V reference
  //ad7689_config |= 0B110 << REF;	// use external reference (maybe ~3.3V)
  ad7689_config |= (sequencer & 0b11) << SEQ;		// don't auto sequence
  ad7689_config |= 0 << RB;		// don't read back config value
  */
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

  Serial.print("\noriginal:      "); Serial.println(ad7689_config, BIN);

  // DEBUG
  //ad7689_config = 0b1111011100000000;
  Serial.print("modified:      "); Serial.println(ad7689_config, BIN);


  pinMode(AD7689_PIN, OUTPUT);      // set the Slave Select Pin as output

  SPI.beginTransaction(AD7689_settings);

  // send config (RAC mode)
  // send twice for dummy conversion
  //for (int i = 0; i < 2; i++) {


    digitalWrite(AD7689_PIN, LOW);
    delayMicroseconds(1); // miniumum 10 ns
    digitalWrite(AD7689_PIN, HIGH);
    delayMicroseconds(4); // minimum 3.2 µs
    digitalWrite(AD7689_PIN, LOW);
    SPI.transfer(ad7689_config >> 8);	// high byte
    SPI.transfer(ad7689_config & 0xFF);	// low byte, 2 bits ignored

    digitalWrite(AD7689_PIN, HIGH);
    SPI.endTransaction();
    //digitalWrite(AD7689_PIN, LOW);
    //delayMicroseconds(1);

    delayMicroseconds(4);


    // change the readback flag back to 1
    uint16_t change_config = ad7689_config;
    ad7689_config = ad7689_config | 0x4;

    SPI.beginTransaction(AD7689_settings);
    digitalWrite(AD7689_PIN, LOW);
    SPI.transfer(ad7689_config >> 8);
    SPI.transfer(ad7689_config & 0xFF);
    digitalWrite(AD7689_PIN, HIGH);
    SPI.endTransaction ();
    //delayMicroseconds(AD_DELAY);
    //}

  delayMicroseconds(4);


  SPI.beginTransaction(AD7689_settings);
    digitalWrite(AD7689_PIN, LOW);
  uint16_t retval = SPI.transfer(ad7689_config >> 8) << 8;
  retval |= SPI.transfer(ad7689_config & 0xFF);
  digitalWrite(AD7689_PIN, HIGH);
  SPI.endTransaction();

  bool changeset = (change_config == retval);


      //Serial.print("disabled:      "); Serial.println(ad7689_config, BIN);
      delayMicroseconds(2); // minumum 1.2µs

/*
SPI.beginTransaction(AD7689_settings);
    //delayMicroseconds(AD_DELAY);
    digitalWrite(AD7689_PIN, LOW);
    SPI.transfer(ad7689_config >> 8);	// high byte
    SPI.transfer(ad7689_config & 0xFF);	// low byte, 2 bits ignored
    digitalWrite(AD7689_PIN, HIGH);
    //delayMicroseconds(AD_DELAY);
SPI.endTransaction();

SPI.beginTransaction(AD7689_settings);
    //delayMicroseconds(AD_DELAY);
    digitalWrite(AD7689_PIN, LOW);
    SPI.transfer(ad7689_config >> 8);	// high byte
    SPI.transfer(ad7689_config & 0xFF);	// low byte, 2 bits ignored
    digitalWrite(AD7689_PIN, HIGH);
    //delayMicroseconds(AD_DELAY);
SPI.endTransaction();
*/
  Serial.print("return config: "); Serial.println(retval, BIN);




  if (changeset) {
    Serial.println("success!");
    Serial.println(retval, HEX);
    return true;
  } else {
    Serial.println("failure");
    return false;
  }


}

// do conversion and return result
// assume that the channel is already set correctly
// sample & hold and conversion starts immediately

//inline uint16_t read_AD7689() __attribute__((always_inline));

uint16_t read_AD7689 ()
{
/*
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
  */
  return 1;
}

float readVoltage(uint8_t AIN) {
  if (AIN != conf.INx_conf) {
    // reading from another channel, reconfigure the ADC
    conf.INx_conf = AIN;
    setConfig();
  }
  return (read_AD7689() * conf.REF_voltage / 65536);
}
