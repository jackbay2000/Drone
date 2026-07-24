#pragma once
#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"

class Position {
public:
  Position();

  void  setup(IMU &imu, FlowSensor &flow, Rangefinder &rangefinder);
  void  update(IMU &imu, FlowSensor &flow, Rangefinder &rangefinder);

  float getX() const;  // meters, takeoff-heading body frame
  float getY() const;  // meters, takeoff-heading body frame
  float getZ() const;  // meters, altitude (tilt-corrected, low-pass filtered)

  // True once the rangefinder has gone too long without a fresh, valid
  // reading (see ALT_STALE_TIMEOUT in Position.cpp) -- getZ() is still
  // returning its last known value at that point, not a live one.
  bool  altitudeStale() const;

  void  reset();       // zeroes X and Y; Z follows rangefinder automatically

private:
  float _x = 0.0f;
  float _y = 0.0f;
  float _z = 0.0f;
  float _timeSinceValidRange = 0.0f;

  unsigned long _lastMicros = 0;
};
