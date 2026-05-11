#pragma once
#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"

class Position {
public:
  Position();

  void  setup(IMU &imu, FlowSensor &flow, Rangefinder &rangefinder);
  void  update(IMU &imu, FlowSensor &flow, Rangefinder &rangefinder);

  float getX() const;  // metres, world frame
  float getY() const;  // metres, world frame
  float getZ() const;  // metres, altitude (tilt-corrected, low-pass filtered)

  void  reset();       // zeroes X and Y; Z follows rangefinder automatically

private:
  float _x = 0.0f;
  float _y = 0.0f;
  float _z = 0.0f;

  unsigned long _lastMicros = 0;
};
