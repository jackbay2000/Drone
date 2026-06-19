#include "PositionPID.h"

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

ControlVector PositionPID::update(float tx, float ty, float tz,
                                  float px, float py, float pz, float dt) {
  ControlVector cv;
  cv.x   = _pidX.compute(tx - px, px, dt);
  cv.y   = _pidY.compute(ty - py, py, dt);
  cv.z   = _pidZ.compute(tz - pz, pz, dt);
  cv.yaw = _desiredYaw;
  return cv;
}
