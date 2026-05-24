#include "controller.h"
#include <math.h>

Controller::Controller()
  : _pidRoll (0, 0, 0, -0.5f,     0.5f)
  , _pidPitch(0, 0, 0, -0.5f,     0.5f)
  , _pidYaw  (0, 0, 0, -0.5f,     0.5f)
  , _pidX    (0, 0, 0, -MAX_TILT, MAX_TILT)
  , _pidY    (0, 0, 0, -MAX_TILT, MAX_TILT)
  , _pidZ    (0, 0, 0, -0.4f,     0.4f)
{}

void Controller::setup(const Gains& g) {
  _pidRoll .setGains(g.kp_roll,  g.ki_roll,  g.kd_roll);
  _pidPitch.setGains(g.kp_pitch, g.ki_pitch, g.kd_pitch);
  _pidYaw  .setGains(g.kp_yaw,   g.ki_yaw,   g.kd_yaw);
  _pidX    .setGains(g.kp_x,     g.ki_x,     g.kd_x);
  _pidY    .setGains(g.kp_y,     g.ki_y,     g.kd_y);
  _pidZ    .setGains(g.kp_z,     g.ki_z,     g.kd_z);
  _esc1.attach(PIN_M1, 1000, 2000);
  _esc2.attach(PIN_M2, 1000, 2000);
  _esc3.attach(PIN_M3, 1000, 2000);
  _esc4.attach(PIN_M4, 1000, 2000);

  // Hold disarm signal — ESCs need 1000 µs for ~2 s before they will respond
  _esc1.writeMicroseconds(1000);
  _esc2.writeMicroseconds(1000);
  _esc3.writeMicroseconds(1000);
  _esc4.writeMicroseconds(1000);
  delay(2000);

  Serial.println("ESCs ready");
  _lastMicros = _lastPositionMicros = micros();
}

void Controller::update(IMU& imu, Position& pos) {
  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1.0e-6f;
  _lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  // Safety cut: beyond 45° the drone is unrecoverable — stop all motors
  if (fabsf(imu.getRoll()) > 0.785f || fabsf(imu.getPitch()) > 0.785f) {
    _esc1.writeMicroseconds(1000);
    _esc2.writeMicroseconds(1000);
    _esc3.writeMicroseconds(1000);
    _esc4.writeMicroseconds(1000);
    return;
  }

  // Outer position loop — runs at POSITION_HZ
  float dt_pos = (now - _lastPositionMicros) * 1.0e-6f;
  if (dt_pos >= 1.0f / POSITION_HZ) {
    _updatePosition(imu, pos, dt_pos);
    _lastPositionMicros = now;
  }

  // Inner attitude loop — runs every call
  _updateAttitude(imu, dt);
}

void Controller::_updatePosition(IMU& imu, Position& pos, float dt) {
  if (_waypointCount == 0 || _currentWaypoint >= _waypointCount) return;

  const Waypoint& target = _waypoints[_currentWaypoint];

  float ex = target.x - pos.getX();
  float ey = target.y - pos.getY();
  float ez = target.z - pos.getZ();

  // Advance to next waypoint when within arrival radius
  if (sqrtf(ex*ex + ey*ey + ez*ez) < WAYPOINT_RADIUS) {
    _currentWaypoint++;
    _pidX.reset(); _pidY.reset(); _pidZ.reset();
    if (_currentWaypoint >= _waypointCount) {
      // All done — level off and hold altitude
      _desiredRoll  = 0.0f;
      _desiredPitch = 0.0f;
      return;
    }
  }

  // ── Position → attitude commands ──────────────────────────────────────
  // Waypoints and position are both in the drone's takeoff-heading frame,
  // so errors are already body-frame — no yaw rotation needed.
  // Positive forward error  → pitch forward  (+pitch = nose down → moves fwd)
  // Positive leftward error → roll left      (−roll  = left down → moves left)
  _desiredPitch = _pidX.compute(ex, pos.getX(), dt);
  _desiredRoll  = -_pidY.compute(ey, pos.getY(), dt);
  _baseThrottle = _clamp(HOVER_THROTTLE + _pidZ.compute(ez, pos.getZ(), dt), 0.0f, 1.0f);
}

void Controller::_updateAttitude(IMU& imu, float dt) {
  float rollErr  = _desiredRoll  - imu.getRoll();
  float pitchErr = _desiredPitch - imu.getPitch();
  float yawErr   = _desiredYaw   - imu.getYaw();

  // Wrap yaw error into [-π, π] so the drone always takes the short way round
  while (yawErr >  3.14159f) yawErr -= 6.28318f;
  while (yawErr < -3.14159f) yawErr += 6.28318f;

  float rollCmd  = _pidRoll.compute (rollErr,  imu.getRoll(),  dt);
  float pitchCmd = _pidPitch.compute(pitchErr, imu.getPitch(), dt);
  float yawCmd   = _pidYaw.compute  (yawErr,   imu.getYaw(),   dt);

  _writeMotors(rollCmd, pitchCmd, yawCmd, _baseThrottle);
}

void Controller::_writeMotors(float roll, float pitch, float yaw, float throttle) {
  // X configuration motor mix
  //
  //   Roll right  (+roll) : left motors faster  → left side lifts → tilts right
  //   Pitch fwd   (+pitch): rear motors faster  → rear lifts → nose drops → moves fwd
  //   Yaw CW      (+yaw)  : CCW motors faster   → reaction torque spins frame CW
  //
  //   M1 (FL, CCW) = throttle + roll − pitch + yaw
  //   M2 (FR, CW)  = throttle − roll − pitch − yaw
  //   M3 (RL, CW)  = throttle + roll + pitch − yaw
  //   M4 (RR, CCW) = throttle − roll + pitch + yaw

  _esc1.writeMicroseconds(_pwm(_clamp(throttle + roll - pitch + yaw, 0.0f, 1.0f)));
  _esc2.writeMicroseconds(_pwm(_clamp(throttle - roll - pitch - yaw, 0.0f, 1.0f)));
  _esc3.writeMicroseconds(_pwm(_clamp(throttle + roll + pitch - yaw, 0.0f, 1.0f)));
  _esc4.writeMicroseconds(_pwm(_clamp(throttle - roll + pitch + yaw, 0.0f, 1.0f)));
}

void Controller::addWaypoint(float x, float y, float z) {
  if (_waypointCount < MAX_WAYPOINTS)
    _waypoints[_waypointCount++] = {x, y, z};
}

void Controller::clearWaypoints() {
  _waypointCount   = 0;
  _currentWaypoint = 0;
}

bool Controller::isComplete() const {
  return _waypointCount > 0 && _currentWaypoint >= _waypointCount;
}

float Controller::_clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int Controller::_pwm(float normalized) {
  return 1000 + (int)(normalized * 1000.0f);
}
