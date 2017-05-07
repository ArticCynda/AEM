#ifndef AD7689_H
#define AD7689_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <SPI.h>

// input configuration: bipolar/unipolar, single ended or differential
#define INCC_BIPOLAR_DIFF     (0b000) // 00X
#define INCC_BIPOLAR_COM      (0b010)
#define INCC_TEMP             (0b011)
#define INCC_UNIPOLAR_DIFF    (0b100)// 10X
#define INCC_UNIPOLAR_REF_COM (0b110)
#define INCC_UNIPOLAR_REF_GND (0b111)

// reference voltage (note: decoupling caps required on REF/REFIN when using INT_REF!)
#define INT_REF_25            (0b000)
#define INT_REF_4096          (0b001)
#define EXT_REF_TEMP_ON       (0b010)
#define EXT_REF_TEMP_BUF      (0b011)
#define EXT_REF_TEMP_OFF      (0b110)
#define EXT_REF_TEMP_OFF_BUF  (0b111)

// sequencer configuration (default: no sequencer)
#define SEQ_OFF               (0b00)
#define SEQ_UPDATE            (0b01)
#define SEQ_SCAN_INPUT_TEMP   (0b10)
#define SEQ_SCAN_INPUT        (0b11)

#define MAX_FREQ              (38000000) // 26 ns period @ VDD 5V and VIO 3.3 - 5V

#define UNIPOLAR_MODE         (0)
#define BIPOLAR_MODE          (1)
#define DIFFERENTIAL_MODE     (2)

#define REF_INTERNAL          (0)
#define REF_EXTERNAL          (1)
#define REF_GND               (2)
#define REF_COM               (3)

// bit shifts needed for config register values, from datasheet p. 27 table 11:
#define CFG                   (13)
#define INCC                  (10)
#define INx                   (7)
#define BW                    (6)
#define REF                   (3)
#define SEQ                   (1)
#define RB                    (0)

#define TEMP_REF              (4.096)  // reference voltage to be used for temperature measurement, either 2.5V or 4.096V
#define BASE_TEMP             (25)
#define TEMP_BASE_VOLTAGE     (0.283)
#define TEMP_RICO             (0.001)
#define INTERNAL_25           (2.5)
#define INTERNAL_4096         (4.096)
#define TOTAL_CHANNELS        (8)
#define TOTAL_STEPS           (65536)
#define TCONV                 (4)

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

// set up the speed, mode and endianness of each device
// MODE0: SCLK idle low (CPOL=0), MOSI read on rising edge (CPHI=0)
// use CPHA = CPOL = 0
// two dummy conversions are required on startup
/*
#if (F_CPU >= MAX_FREQ)
  SPISettings AD7689_settings (MAX_FREQ, MSBFIRST, SPI_MODE0); // set SPI clock to maximum (38 MHz default)
#else
  //SPISettings AD7689_settings (F_CPU, MSBFIRST, SPI_MODE0); // set SPI clock to CPU clock
  SPISettings AD7689_settings (1000000, MSBFIRST, SPI_MODE0); // set SPI clock to CPU clock
#endif
*/

//AD7689::AD7689(uint8_t SSpin, uint8_t refSource, float ref);

class AD7689 {
  protected:
    AD7689_conf conf;
    bool init_complete = false;

    const SPISettings AD7689_settings;

    // Supports highly accurate sample time
    uint8_t AD7689_PIN;		// chip select pin to use (10 is standard)
    float posref;
    float negref;

    uint32_t timeStamps[TOTAL_CHANNELS]; // time stamps for each sample
    uint16_t samples[TOTAL_CHANNELS];
    uint16_t framePeriod;
    uint16_t curTemp;
    uint16_t tempTime;

    uint8_t refsrc;

    uint8_t inputConfig;
    uint8_t inputCount;
    uint8_t refConfig;
    bool filterConfig;

    uint16_t shiftTransaction(uint16_t command, bool readback, uint16_t* rb_cmd_ptr);
    uint16_t toCommand(AD7689_conf cfg) const;
    AD7689_conf getADCConfig(void) const;
    AD7689_conf getDefaultConfig(void) const;

    float readTemperature(void);

    void configureSequencer();
    void readChannels(uint8_t channels, uint8_t mode, uint16_t* data, uint16_t* temp);

    float calculateVoltage(uint16_t sample, float posRef, float negRef);
    float calculateTemp(uint16_t temp);

    uint32_t initSampleTiming(void);
    void initializeTiming(void);

  public:

    // configure ADC
    AD7689(uint8_t SSpin, uint8_t numberChannels = TOTAL_CHANNELS);
    void setReference(uint8_t refSource, float posRef, uint8_t polarity, bool differential);

    void enableFiltering(bool onOff);

    float acquireChannel(uint8_t channel, uint32_t* timeStamp);
    float acquireTemperature();

    bool selftest(void);
};
#endif
