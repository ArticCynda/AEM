#ifndef YSPI_H
#define YSPI_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//class YMSPI public: YGeneralSpi{
class YMSPI {
 public:
  YMSPI(uint8_t id);
  uint8_t MSPIMTransfer (uint8_t c);

};



#endif
