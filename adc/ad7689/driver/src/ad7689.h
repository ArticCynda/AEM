#ifndef AD7689_H
#define AD7689_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
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

class AD7689 {
  protected:
    AD7689_conf conf;
    bool init_complete = false;

    const SPISettings AD7689_settings;

    // Supports highly accurate sample time
    uint8_t AD7689_PIN;		// chip select pin to use (10 is standard)

    void init(float vref);
    uint16_t shiftTransaction(uint16_t command, bool readback, uint16_t* rb_cmd_ptr);
    uint16_t toCommand(AD7689_conf cfg) const;
    AD7689_conf getDefaultConfig(void) const;

    float readTemperature(void);

  public:
    void configureSequencer(AD7689_conf sequence);
    void readChannels(uint8_t channels, uint8_t mode, uint16_t* data, uint16_t* temp);
    //uint16_t read_AD7689 (void) const;
    //float readVoltage(uint8_t AIN);
    //void setConfig(void);
    bool selftest(void);
    AD7689(uint8_t SSpin, float vref);
    void init(uint8_t SSpin, uint8_t refSource, float ref);
};

#endif