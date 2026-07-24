#pragma once
#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"

// ---------------------------------------------------------------------------
// Position stub — x/y/z are injected from Python via sim_set_state() each step.
// Exposes the same public API that PositionPID uses.
// ---------------------------------------------------------------------------
class Position {
public:
    Position() : _x(0), _y(0), _z(0) {}

    void setup(IMU&, FlowSensor&, Rangefinder&) {}
    void update(IMU&, FlowSensor&, Rangefinder&) {}
    void reset() { _x = _y = _z = 0; }

    // Setters called from sim_bridge.cpp
    void setX(float v) { _x = v; }
    void setY(float v) { _y = v; }
    void setZ(float v) { _z = v; }
    void setAltitudeStale(bool s) { _stale = s; }

    // Getters used by PositionPID / Controller
    float getX() const { return _x; }
    float getY() const { return _y; }
    float getZ() const { return _z; }
    bool  altitudeStale() const { return _stale; }

private:
    float _x, _y, _z;
    bool  _stale = false;
};
