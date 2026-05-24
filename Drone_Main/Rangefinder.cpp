#include "Rangefinder.h"

Rangefinder::Rangefinder() {}

void Rangefinder::setup() {
  if (!_tof.begin(0x29, &Wire)) {
    Serial.println("VL53L1X not found — check I2C wiring");
    while (1) delay(500);
  }
  _tof.setTimingBudget(20);  // 20 ms per measurement — up to 50 Hz
  _tof.startRanging();
  Serial.println("VL53L1X ready");
}

void Rangefinder::update() {
  if (!_tof.dataReady()) return;

  int16_t d = _tof.distance();
  _tof.clearInterrupt();

  if (d > 0 && d < 4000) {
    _distance = d * 0.001f;  // mm to meters
    _hasData  = true;
  }
}

float Rangefinder::getDistance() const { return _distance; }
bool  Rangefinder::hasData()     const { return _hasData;  }
