#pragma once
#include <Bitcraze_PMW3901.h>

class FlowSensor {
public:
  explicit FlowSensor(uint8_t csPin);

  void setup();
  void read(int16_t &dx, int16_t &dy);

private:
  Bitcraze_PMW3901 _flow;
};
