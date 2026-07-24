#include "Rangefinder.h"

Rangefinder::Rangefinder() {}

void Rangefinder::setup() {
  if (!_tof.begin(0x29, &Wire)) {
    Serial.println("VL53L1X not found — check I2C wiring");
    while (1) delay(500);
  }
  // Adafruit_VL53L1X::begin() leaves the sensor in LONG distance mode (the
  // ST driver's default), but a 20ms timing budget is only valid in SHORT
  // mode per the datasheet (33ms is the minimum for any other mode, 140ms
  // recommended for LONG mode's full 4m range) -- so this was previously an
  // out-of-spec combination. Explicitly select SHORT mode: it matches the
  // 20ms budget we want for a 50Hz refresh, and its ambient-light immunity
  // is a good fit for a mission that never flies above 0.4m anyway (SHORT
  // mode's ~1.3m max range comfortably covers that with margin). Diagnosed
  // 2026-07-19 after real dropouts starting ~0.65m -- see project memory.
  _tof.VL53L1X_SetDistanceMode(1);  // 1 = short, 2 = long (was left at default)
  _tof.setTimingBudget(20);  // 20 ms per measurement — valid for short mode

  // Timing budget alone only controls how LONG one measurement takes, not
  // how OFTEN one starts -- that's the inter-measurement period, which
  // Adafruit_VL53L1X never sets (confirmed in its source: begin(),
  // setTimingBudget(), and startRanging() never touch it) and which
  // defaults to 100ms (~10Hz) per the ST driver. Left alone, real refresh
  // was likely ~10Hz despite the 20ms budget, not the ~50Hz this file
  // assumed -- explicitly matching it to the timing budget gets the real
  // ~50Hz cadence the altitude loop (and Physics_Sim/sim/timing.py's
  // RANGEFINDER_HZ=50) was already assuming. Must be >= timing budget.
  _tof.VL53L1X_SetInterMeasurementInMs(20);
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
