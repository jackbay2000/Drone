#pragma once

// ---------------------------------------------------------------------------
// FlowSensor stub — shadows Drone_Main/FlowSensor.h (which pulls in the real
// Bitcraze_PMW3901 hardware library). Position is injected directly via
// sim_set_state() in the sim, so this stub is never actually read from.
// ---------------------------------------------------------------------------

class FlowSensor {
public:
    explicit FlowSensor(uint8_t /*csPin*/) {}

    void setup() {}
    void read(int16_t &dx, int16_t &dy) { dx = 0; dy = 0; }
};
