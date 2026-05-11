#pragma once

class PID {
public:
  PID(float kp, float ki, float kd, float outMin, float outMax);

  // error       : setpoint - measurement
  // measurement : raw sensor value — derivative is taken on this, not the error,
  //               to avoid output spikes when the setpoint changes suddenly
  // dt          : seconds since last call
  float compute(float error, float measurement, float dt);

  void  reset();
  void  setGains(float kp, float ki, float kd);

private:
  float _kp, _ki, _kd;
  float _integral;
  float _prevMeasurement;
  float _outMin, _outMax;
  bool  _firstRun;

  static float _clamp(float v, float lo, float hi);
};
