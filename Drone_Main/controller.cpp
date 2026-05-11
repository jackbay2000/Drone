#include "controller.h"
#include <math.h>

// ── Attitude PID gains ─────────────────────────────────────────────────────
// Tune these first, on the bench with props OFF, verifying that the attitude
// error outputs look correct before installing propellers.
static constexpr float KP_ROLL  = 0.0f, KI_ROLL  = 0.0f, KD_ROLL  = 0.0f;
static constexpr float KP_PITCH = 0.0f, KI_PITCH = 0.0f, KD_PITCH = 0.0f;
static constexpr float KP_YAW   = 0.0f, KI_YAW   = 0.0f, KD_YAW   = 0.0f;

// ── Position PID gains ─────────────────────────────────────────────────────
// Tune these only after attitude is solid.  Start with kp only; add kd if
// the drone overshoots waypoints; add ki only if it consistently stops short.
static constexpr float KP_X = 0.0f, KI_X = 0.0f, KD_X = 0.0f;
static constexpr float KP_Y = 0.0f, KI_Y = 0.0f, KD_Y = 0.0f;
static constexpr float KP_Z = 0.0f, KI_Z = 0.0f, KD_Z = 0.0f;

Controller::Controller()
  // Attitude — output limits in normalised motor delta [-0.5, 0.5]
  : _pidRoll (KP_ROLL,  KI_ROLL,  KD_ROLL,  -0.5f, 0.5f)
  , _pidPitch(KP_PITCH, KI_PITCH, KD_PITCH, -0.5f, 0.5f)
  , _pidYaw  (KP_YAW,   KI_YAW,   KD_YAW,   -0.5f, 0.5f)
  // Position — X/Y output in radians (tilt angle), Z output as throttle delta
  , _pidX(KP_X, KI_X, KD_X, -MAX_TILT, MAX_TILT)
  , _pidY(KP_Y, KI_Y, KD_Y, -MAX_TILT, MAX_TILT)
  , _pidZ(KP_Z, KI_Z, KD_Z,     -0.4f,    0.4f)
{}

void Controller::setup() {
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

  // ── Rotate world-frame error into body frame ───────────────────────────
  // Waypoints are stored in the world frame (fixed at takeoff).
  // Roll/pitch commands are in the body frame (rotates with the drone).
  // Multiplying by the inverse yaw rotation converts between them.
  float cy = cosf(imu.getYaw());
  float sy = sinf(imu.getYaw());
  float ex_b =  ex * cy + ey * sy;   // body forward error  (+X = forward)
  float ey_b = -ex * sy + ey * cy;   // body lateral error  (+Y = left)

  // ── Position → attitude commands ──────────────────────────────────────
  // Positive forward error  → pitch forward  (+pitch = nose down → moves fwd)
  // Positive leftward error → roll left      (−roll  = left down → moves left)
  _desiredPitch = _pidX.compute(ex_b, pos.getX(), dt);
  _desiredRoll  = -_pidY.compute(ey_b, pos.getY(), dt);
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

int Controller::_pwm(float normalised) {
  return 1000 + (int)(normalised * 1000.0f);
}
