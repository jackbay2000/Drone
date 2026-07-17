#pragma once
#include "PID.h"

struct ControlVector { float x, y, z; };

class PositionPID {
public:
  PositionPID();

  void setup(float kp_x, float ki_x, float kd_x,
             float kp_y, float ki_y, float kd_y,
             float kp_z, float ki_z, float kd_z);

  ControlVector update(float tx, float ty, float tz,
                       float px, float py, float pz, float dt);

private:
  PID _pidX, _pidY, _pidZ;
  static constexpr float MAX_TILT = 0.1396f;   // ~8 deg -- slow/trackable flight cap
};
