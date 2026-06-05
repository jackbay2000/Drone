#pragma once
#include "PID.h"
#include "Position.h"

// Desired motion direction produced by the position loop.
// x/y are tilt-angle commands in radians; z is a normalized throttle delta.
struct ControlVector {
  float x;    // forward motion command [−MAX_TILT, MAX_TILT] rad
  float y;    // leftward motion command [−MAX_TILT, MAX_TILT] rad
  float z;    // throttle delta, normalized [−0.4, 0.4]
  float yaw;  // desired heading, radians (absolute, locked at takeoff)
};

struct Waypoint {
  float x, y, z;   // meters, body frame, relative to start position
};

class PositionPID {
public:
  PositionPID();

  void setup(float kp_x, float ki_x, float kd_x,
             float kp_y, float ki_y, float kd_y,
             float kp_z, float ki_z, float kd_z);

  // Compute position error against the current waypoint and return motion commands.
  // Call at POSITION_HZ (25 Hz). Advances to the next waypoint automatically.
  ControlVector update(const Position& pos, float dt);

  // Populate the waypoint list before calling update()
  void addWaypoint(float x, float y, float z);
  void clearWaypoints();
  bool isComplete() const;   // true once all waypoints have been reached
  void reset();

private:
  // Forward error (m) → desired pitch (rad)
  // Leftward error (m) → leftward motion magnitude (rad)
  // Altitude error (m) → throttle delta
  PID _pidX;
  PID _pidY;
  PID _pidZ;

  static constexpr int   MAX_WAYPOINTS   = 20;
  static constexpr float WAYPOINT_RADIUS = 0.15f;   // m — arrival threshold
  static constexpr float MAX_TILT        = 0.35f;   // rad (~20 deg) — caps X/Y commands

  Waypoint _waypoints[MAX_WAYPOINTS];
  int _waypointCount   = 0;
  int _currentWaypoint = 0;

  float _desiredYaw = 0.0f;   // locked at takeoff heading; no in-flight yaw changes
};
