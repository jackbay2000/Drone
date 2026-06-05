#include "PositionPID.h"
#include <math.h>

PositionPID::PositionPID()
  : _pidX(0, 0, 0, -MAX_TILT, MAX_TILT)
  , _pidY(0, 0, 0, -MAX_TILT, MAX_TILT)
  , _pidZ(0, 0, 0, -0.4f,     0.4f) {}

void PositionPID::setup(float kp_x, float ki_x, float kd_x,
                        float kp_y, float ki_y, float kd_y,
                        float kp_z, float ki_z, float kd_z) {
  _pidX.setGains(kp_x, ki_x, kd_x);
  _pidY.setGains(kp_y, ki_y, kd_y);
  _pidZ.setGains(kp_z, ki_z, kd_z);
}

ControlVector PositionPID::update(const Position& pos, float dt) {
  ControlVector cv = {0.0f, 0.0f, 0.0f, _desiredYaw};

  if (_waypointCount == 0 || _currentWaypoint >= _waypointCount) return cv;

  const Waypoint& target = _waypoints[_currentWaypoint];

  float ex = target.x - pos.getX();
  float ey = target.y - pos.getY();
  float ez = target.z - pos.getZ();

  // Advance to the next waypoint when within the arrival radius
  if (sqrtf(ex*ex + ey*ey + ez*ez) < WAYPOINT_RADIUS) {
    _currentWaypoint++;
    _pidX.reset(); _pidY.reset(); _pidZ.reset();
    return cv;
  }

  // Waypoints and position share the same takeoff-heading frame, so no
  // yaw rotation is needed before passing errors to the PIDs.
  // Positive forward error  → positive pitch command (nose tilts forward → advances)
  // Positive leftward error → positive Y command (sign inverted to roll in AttitudeController)
  cv.x = _pidX.compute(ex, pos.getX(), dt);
  cv.y = _pidY.compute(ey, pos.getY(), dt);
  cv.z = _pidZ.compute(ez, pos.getZ(), dt);
  return cv;
}

void PositionPID::addWaypoint(float x, float y, float z) {
  if (_waypointCount < MAX_WAYPOINTS)
    _waypoints[_waypointCount++] = {x, y, z};
}

void PositionPID::clearWaypoints() {
  _waypointCount   = 0;
  _currentWaypoint = 0;
}

bool PositionPID::isComplete() const {
  return _waypointCount > 0 && _currentWaypoint >= _waypointCount;
}

void PositionPID::reset() {
  _pidX.reset(); _pidY.reset(); _pidZ.reset();
  _waypointCount   = 0;
  _currentWaypoint = 0;
}
