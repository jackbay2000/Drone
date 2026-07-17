#pragma once

// ---------------------------------------------------------------------------
// Rangefinder stub — shadows Drone_Main/Rangefinder.h (which pulls in the
// real Adafruit_VL53L1X hardware library). Altitude is injected directly via
// sim_set_state() in the sim, so this stub is never actually read from.
// ---------------------------------------------------------------------------

class Rangefinder {
public:
    Rangefinder() {}

    void  setup() {}
    void  update() {}

    float getDistance() const { return 0.0f; }
    bool  hasData()     const { return true; }
};
