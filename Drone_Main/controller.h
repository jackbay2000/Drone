#pragma once
#include <Servo.h>
#include "IMU.h"
#include "Position.h"
#include "Gains.h"
#include "PositionPID.h"
#include "AttitudeController.h"
#include "YawController.h"

class Controller {
public:
  Controller();

  void setup(const Gains& g);
  void update(IMU& imu, Position& pos);

  // Populate the waypoint list before calling setup()
  void addWaypoint(float x, float y, float z);
  void clearWaypoints();
  bool isComplete() const;   // true once all waypoints have been reached

private:
  PositionPID        _posPID;    // outer loop: position error → motion commands
  AttitudeController _attCtrl;   // inner loop: motion commands → roll/pitch commands
  YawController      _yawCtrl;   // yaw loop:   heading error  → yaw moment

  // X configuration viewed from above (front of drone at top):
  //
  //   M1 (FL, CCW) ──── M2 (FR, CW)
  //         \                /
  //          \              /
  //   M3 (RL, CW)  ──── M4 (RR, CCW)
  static constexpr uint8_t PIN_M1 = 2;   // Front-Left,  CCW
  static constexpr uint8_t PIN_M2 = 3;   // Front-Right, CW
  static constexpr uint8_t PIN_M3 = 4;   // Rear-Left,   CW
  static constexpr uint8_t PIN_M4 = 5;   // Rear-Right,  CCW

  Servo _esc1, _esc2, _esc3, _esc4;

  // HOVER_THROTTLE: normalized [0,1]. Raise until the drone just lifts off level.
  static constexpr float HOVER_THROTTLE = 0.5f;
  // POSITION_HZ: outer loop rate. Attitude and yaw loops run every update() call.
  static constexpr float POSITION_HZ    = 25.0f;

  // Motion commands held between position loop ticks for the attitude/yaw loops
  float _cvX   = 0.0f;   // forward command (→ desired pitch)
  float _cvY   = 0.0f;   // leftward command (→ desired roll, sign inverted)
  float _cvYaw = 0.0f;   // desired heading, radians

  float _baseThrottle = HOVER_THROTTLE;

  unsigned long _lastMicros         = 0;
  unsigned long _lastPositionMicros = 0;

  // Mix normalized roll/pitch/yaw/throttle into ESC PWM signals
  void _writeMotors(float roll, float pitch, float yaw, float throttle);

  static float _clamp(float v, float lo, float hi);
  static int   _pwm(float normalized);   // [0,1] → [1000,2000] µs
};
