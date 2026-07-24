#include "controller.h"
#include <math.h>

Controller::Controller() {}

void Controller::setup(const Gains& g) {
  _posPID.setup(g.kp_x, g.ki_x, g.kd_x,
                g.kp_y, g.ki_y, g.kd_y,
                g.kp_z, g.ki_z, g.kd_z);

  _attCtrl.setup(g.kp_roll, g.ki_roll, g.kd_roll,
                 g.kp_pitch, g.ki_pitch, g.kd_pitch);

  _yawCtrl.setup(g.kp_yaw, g.ki_yaw, g.kd_yaw);

  const uint8_t pins[] = {2, 3, 4, 5};
  for (int i = 0; i < 4; i++) {
    _esc[i].attach(pins[i], 1000, 2000);
    _esc[i].writeMicroseconds(1000);
  }
  delay(2000);

  _lastMicros = _lastPositionMicros = micros();
}

void Controller::update(IMU& imu, Position& pos, float tx, float ty, float tz, float targetYaw) {
  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1.0e-6f;
  _lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  if (fabsf(imu.getRoll()) > 0.785f || fabsf(imu.getPitch()) > 0.785f) {
    halt();
    return;
  }

  // Same fail-safe pattern as the tilt cut above: if altitude data has been
  // stale too long (see Position::altitudeStale()), the position loop would
  // be flying on a frozen Z estimate with no way to know it's wrong. Cutting
  // power is the safe default here, same as an excessive tilt.
  if (pos.altitudeStale()) {
    halt();
    return;
  }

  _cvYaw = targetYaw;

  // Position loop at POSITION_HZ
  float dt_pos = (now - _lastPositionMicros) * 1.0e-6f;
  if (dt_pos >= 1.0f / POSITION_HZ) {
    _lastPositionMicros = now;
    ControlVector cv = _posPID.update(tx, ty, tz,
                                      pos.getX(), pos.getY(), pos.getZ(), dt_pos);
    _cvX          = cv.x;
    _cvY          = cv.y;
    _baseThrottle = _clamp(HOVER_THROTTLE + cv.z, 0.0f, 1.0f);
  }

  // Attitude loop at full rate. _cvX/_cvY are world-frame (PositionPID
  // operates on Position's world-frame x/y against fixed waypoint targets).
  // Rotate into body frame by -yaw before treating them as pitch/roll tilt
  // commands -- otherwise "pitch forward" moves the vehicle in its current
  // heading direction, not toward the world-frame target, and any
  // keep_heading=false leg (nonzero target yaw) can spiral away.
  float yawNow = imu.getYaw();
  float cosY = cosf(yawNow), sinY = sinf(yawNow);
  float bodyX =  _cvX * cosY + _cvY * sinY;
  float bodyY = -_cvX * sinY + _cvY * cosY;

  AttitudeOutput att = _attCtrl.update(imu, bodyX, bodyY, dt);
  float yawCmd       = _yawCtrl.update(imu, _cvYaw, dt);

  _writeMotors(att.roll, att.pitch, yawCmd, _baseThrottle);
}

void Controller::_writeMotors(float roll, float pitch, float yaw, float throttle) {
  float m[4] = {
    _clamp(throttle + roll - pitch + yaw, 0.0f, 1.0f),
    _clamp(throttle - roll - pitch - yaw, 0.0f, 1.0f),
    _clamp(throttle + roll + pitch - yaw, 0.0f, 1.0f),
    _clamp(throttle - roll + pitch + yaw, 0.0f, 1.0f)
  };
  for (int i = 0; i < 4; i++) _esc[i].writeMicroseconds(_pwm(m[i]));
}

void Controller::halt() {
  for (int i = 0; i < 4; i++) _esc[i].writeMicroseconds(1000);
}

float Controller::_clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int Controller::_pwm(float normalized) {
  return 1000 + (int)(normalized * 1000.0f);
}
