#pragma once
#include <Wire.h>
#include <MPU6050.h>

class IMU {
public:
  IMU();
  void  setup();
  void  update();

  float getRoll()  const;
  float getPitch() const;
  float getYaw()   const;

  // Gyro rates in rad/s — used by Position to correct optical flow for rotation
  float getGX() const;
  float getGY() const;
  float getGZ() const;

private:
  MPU6050       _imu;
  unsigned long _lastMicros = 0;

  float _accelBias[3] = {};
  float _gyroBias[3]  = {};

  float _roll  = 0.0f;
  float _pitch = 0.0f;
  float _yaw   = 0.0f;
  float _gRate[3] = {};
};
