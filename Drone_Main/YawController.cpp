#include "YawController.h"
#include <math.h>

YawController::YawController()
  : _pidYaw(0, 0, 0, -0.5f, 0.5f) {}

void YawController::setup(float kp_yaw, float ki_yaw, float kd_yaw) {
  _pidYaw.setGains(kp_yaw, ki_yaw, kd_yaw);
}

float YawController::update(IMU& imu, float desiredYaw, float dt) {
  float err = desiredYaw - imu.getYaw();
  while (err >  3.14159f) err -= 6.28318f;
  while (err < -3.14159f) err += 6.28318f;
  return _pidYaw.compute(err, imu.getYaw(), dt);
}
