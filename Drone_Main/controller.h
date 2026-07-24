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
  void update(IMU& imu, Position& pos, float tx, float ty, float tz, float targetYaw);
  void halt();

private:
  PositionPID        _posPID;
  AttitudeController _attCtrl;
  YawController      _yawCtrl;

  PWMServo _esc[4];

  // Baseline throttle the Z position loop's PID output is added to. Diagnosed
  // 2026-07-24: this was 0.5f, an unvalidated guess -- computed from this
  // vehicle's real bench-measured mass (698g, component_list.json) against
  // the motor thrust curve, actual required hover throttle is only ~0.397.
  // With the old 0.5f baseline, the Z loop had to permanently fight a ~10-
  // percentage-point excess-thrust bias for the entire flight, every leg,
  // for as long as ki_z's slow integral action took to claw it back (often
  // longer than a whole mission) -- this showed up as persistent altitude
  // overshoot/error that MORE tuning trials couldn't fix, because it wasn't
  // a gain-tuning problem, it was a wrong constant. Re-derive the same way
  // if the vehicle's mass or motors ever change materially: (bench mass_kg *
  // 9.81 / 4) run through MotorModel.throttle_from_thrust() in Physics_Sim.
  static constexpr float HOVER_THROTTLE = 0.3974f;
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
