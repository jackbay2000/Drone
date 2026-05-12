#pragma once
#include <Servo.h>
#include "PID.h"
#include "IMU.h"
#include "Position.h"
#include "Gains.h"

struct Waypoint {
  float x, y, z;   // metres, world frame, relative to start position
};

class Controller {
public:
  Controller();

  void setup(const Gains& g);
  void update(IMU& imu, Position& pos);

  // Build the waypoint list before calling setup()
  void addWaypoint(float x, float y, float z);
  void clearWaypoints();
  bool isComplete() const;   // true once all waypoints have been reached

private:
  // ── Inner loop: attitude ──────────────────────────────────────────────────
  // Controls roll, pitch, yaw to match the angles commanded by the position loop.
  // Output is a normalised motor delta [-0.5, 0.5].
  //
  // ALL GAINS START AT ZERO — tune before first flight:
  //   1. Raise kp_roll/pitch until drone holds level but just oscillates
  //   2. Raise kd_roll/pitch to damp the oscillation
  //   3. Add small ki_roll/pitch to remove steady-state lean
  //   4. Repeat for yaw, then altitude, then X/Y position
  PID _pidRoll;
  PID _pidPitch;
  PID _pidYaw;

  // ── Outer loop: position ──────────────────────────────────────────────────
  // Converts X/Y error (metres) into desired tilt angles (radians)
  // and Z error into a throttle delta.  Runs at POSITION_HZ.
  // Tune only after the attitude loop is solid.
  PID _pidX;
  PID _pidY;
  PID _pidZ;

  // ── Waypoints ─────────────────────────────────────────────────────────────
  static constexpr int   MAX_WAYPOINTS   = 20;
  static constexpr float WAYPOINT_RADIUS = 0.15f;   // m — arrival threshold

  Waypoint _waypoints[MAX_WAYPOINTS];
  int _waypointCount   = 0;
  int _currentWaypoint = 0;

  // ── Motors ────────────────────────────────────────────────────────────────
  // X configuration viewed from above (front of drone at top):
  //
  //   M1(FL,CCW) ---- M2(FR,CW)
  //         X              X
  //         X              X
  //   M3(RL,CW)  ---- M4(RR,CCW)
  //
  // Adjust PIN_M1–M4 to match your physical wiring.
  static constexpr uint8_t PIN_M1 = 2;   // Front-Left,  CCW
  static constexpr uint8_t PIN_M2 = 3;   // Front-Right, CW
  static constexpr uint8_t PIN_M3 = 4;   // Rear-Left,   CW
  static constexpr uint8_t PIN_M4 = 5;   // Rear-Right,  CCW

  Servo _esc1, _esc2, _esc3, _esc4;

  // ── Flight parameters ─────────────────────────────────────────────────────
  // HOVER_THROTTLE: normalised [0,1]. Raise until drone just lifts off level.
  static constexpr float HOVER_THROTTLE = 0.5f;
  // MAX_TILT: maximum commanded lean in radians (~20°). Caps XY speed.
  static constexpr float MAX_TILT       = 0.35f;
  // POSITION_HZ: outer position loop rate. Inner attitude loop runs every update().
  static constexpr float POSITION_HZ    = 25.0f;

  // Desired attitude produced by position loop, consumed by attitude loop
  float _desiredRoll     = 0.0f;
  float _desiredPitch    = 0.0f;
  float _desiredYaw      = 0.0f;   // fixed at takeoff heading
  float _baseThrottle    = HOVER_THROTTLE;

  unsigned long _lastMicros         = 0;
  unsigned long _lastPositionMicros = 0;

  void _updatePosition(IMU& imu, Position& pos, float dt);
  void _updateAttitude(IMU& imu, float dt);
  void _writeMotors(float roll, float pitch, float yaw, float throttle);

  static float _clamp(float v, float lo, float hi);
  static int   _pwm(float normalised);   // [0,1] → [1000,2000] µs
};
