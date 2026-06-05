#include "FlightController.h"
#include <math.h>

FlightController::FlightController() {}

void FlightController::setup(const Gains& g) {
  _posPID.setup(g.kp_x,     g.ki_x,     g.kd_x,
                g.kp_y,     g.ki_y,     g.kd_y,
                g.kp_z,     g.ki_z,     g.kd_z);

  _attCtrl.setup(g.kp_roll,  g.ki_roll,  g.kd_roll,
                 g.kp_pitch, g.ki_pitch, g.kd_pitch);

  _yawCtrl.setup(g.kp_yaw, g.ki_yaw, g.kd_yaw);

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

  Serial.println("FlightController ready");
  _lastMicros = _lastPositionMicros = micros();
}

void FlightController::update(IMU& imu, Position& pos) {
  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1.0e-6f;
  _lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  // Safety cut: beyond 45° the drone is unrecoverable — kill all motors
  if (fabsf(imu.getRoll()) > 0.785f || fabsf(imu.getPitch()) > 0.785f) {
    _esc1.writeMicroseconds(1000);
    _esc2.writeMicroseconds(1000);
    _esc3.writeMicroseconds(1000);
    _esc4.writeMicroseconds(1000);
    return;
  }

  // Outer position loop — runs at POSITION_HZ (25 Hz)
  float dt_pos = (now - _lastPositionMicros) * 1.0e-6f;
  if (dt_pos >= 1.0f / POSITION_HZ) {
    ControlVector cv = _posPID.update(pos, dt_pos);
    _cvX         = cv.x;
    _cvY         = cv.y;
    _cvYaw       = cv.yaw;
    _baseThrottle = _clamp(HOVER_THROTTLE + cv.z, 0.0f, 1.0f);
    _lastPositionMicros = now;
  }

  // Inner attitude loop — runs every call (full IMU rate)
  AttitudeOutput att = _attCtrl.update(imu, _cvX, _cvY, dt);
  float yawCmd       = _yawCtrl.update(imu, _cvYaw, dt);

  _writeMotors(att.roll, att.pitch, yawCmd, _baseThrottle);
}

void FlightController::_writeMotors(float roll, float pitch, float yaw, float throttle) {
  // Motor mix — X configuration:
  //
  //   Roll right (+roll) : left motors faster  → left side lifts → frame tilts right
  //   Pitch fwd  (+pitch): rear motors faster  → rear lifts → nose drops → moves fwd
  //   Yaw CW     (+yaw)  : CCW motors (M1, M4) faster → net CCW aero torque → frame reacts CW
  //
  //   M1 (FL, CCW): throttle + roll − pitch + yaw
  //   M2 (FR, CW) : throttle − roll − pitch − yaw
  //   M3 (RL, CW) : throttle + roll + pitch − yaw
  //   M4 (RR, CCW): throttle − roll + pitch + yaw

  _esc1.writeMicroseconds(_pwm(_clamp(throttle + roll - pitch + yaw, 0.0f, 1.0f)));
  _esc2.writeMicroseconds(_pwm(_clamp(throttle - roll - pitch - yaw, 0.0f, 1.0f)));
  _esc3.writeMicroseconds(_pwm(_clamp(throttle + roll + pitch - yaw, 0.0f, 1.0f)));
  _esc4.writeMicroseconds(_pwm(_clamp(throttle - roll + pitch + yaw, 0.0f, 1.0f)));
}

void FlightController::addWaypoint(float x, float y, float z) {
  _posPID.addWaypoint(x, y, z);
}

void FlightController::clearWaypoints() {
  _posPID.clearWaypoints();
}

bool FlightController::isComplete() const {
  return _posPID.isComplete();
}

float FlightController::_clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int FlightController::_pwm(float normalized) {
  return 1000 + (int)(normalized * 1000.0f);
}
