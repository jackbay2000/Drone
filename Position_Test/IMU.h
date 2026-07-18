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

  // Raw IMU-frame values (no axis remap, no bias subtraction).
  // Gyro in deg/s — whichever changes during a yaw rotation is the vertical axis.
  // Accel in g   — whichever reads ~1.0 at rest is the vertical axis.
  float rawGX() const;  float rawAX() const;
  float rawGY() const;  float rawAY() const;
  float rawGZ() const;  float rawAZ() const;

private:
  MPU6050       _imu;
  unsigned long _lastMicros = 0;

  float _gyroBias[3]  = {};
  float _rawIMUG[3]   = {};
  float _rawIMUA[3]   = {};

  float _roll  = 0.0f;
  float _pitch = 0.0f;
  float _yaw   = 0.0f;
  float _gRate[3] = {};

  float _rollOffset  = 0.0f;
  float _pitchOffset = 0.0f;

  float   _accelRollBuf[3]  = {};
  float   _accelPitchBuf[3] = {};
  uint8_t _accelBufIdx      = 0;
};
