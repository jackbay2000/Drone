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

  // Attitude loop at full rate
  AttitudeOutput att = _attCtrl.update(imu, _cvX, _cvY, dt);
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
