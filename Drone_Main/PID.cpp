#include "PID.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
  : _kp(kp), _ki(ki), _kd(kd),
    _integral(0.0f), _prevMeasurement(0.0f),
    _outMin(outMin), _outMax(outMax), _firstRun(true) {}

float PID::compute(float error, float measurement, float dt) {
  if (dt <= 0.0f) return _clamp(_kp * error + _integral, _outMin, _outMax);

  float p = _kp * error;

  // Clamp integral to output limits to prevent windup
  _integral = _clamp(_integral + _ki * error * dt, _outMin, _outMax);

  // Derivative on measurement avoids a spike when setpoint changes
  float d = 0.0f;
  if (!_firstRun)
    d = -_kd * (measurement - _prevMeasurement) / dt;
  _prevMeasurement = measurement;
  _firstRun = false;

  return _clamp(p + _integral + d, _outMin, _outMax);
}

void PID::reset() {
  _integral = 0.0f;
  _firstRun = true;
}

void PID::setGains(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

float PID::_clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
