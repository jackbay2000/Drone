#pragma once
#include <Adafruit_VL53L1X.h>

class Rangefinder {
public:
  Rangefinder();

  void  setup();
  void  update();

  float getDistance() const;  // metres, raw (no tilt correction)
  bool  hasData()     const;  // true once first valid reading received

private:
  Adafruit_VL53L1X _tof;
  float _distance = 0.0f;
  bool  _hasData  = false;
};
