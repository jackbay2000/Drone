#pragma once
#include "main.h"
#include <PWMServo.h>
#include "PositionPID.h"
#include "AttitudeController.h"
#include "YawController.h"

class Controller {
public:
  Controller();

  void setup(const Gains& g);
  void update(IMU& imu, Position& pos, float tx, float ty, float tz);
  void halt();

private:
  PositionPID        _posPID;
  AttitudeController _attCtrl;
  YawController      _yawCtrl;

  PWMServo _esc[4];

  static constexpr float HOVER_THROTTLE = 0.5f;
  static constexpr float POSITION_HZ    = 25.0f;

  float _cvX   = 0.0f;
  float _cvY   = 0.0f;
  float _cvYaw = 0.0f;
  float _baseThrottle = HOVER_THROTTLE;

  unsigned long _lastMicros         = 0;
  unsigned long _lastPositionMicros = 0;

  void _writeMotors(float roll, float pitch, float yaw, float throttle);

  static float _clamp(float v, float lo, float hi);
  static int   _pwm(float normalized);
};
