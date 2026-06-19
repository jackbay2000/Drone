#pragma once
#include "PID.h"
#include "IMU.h"

class YawController {
public:
  YawController();

  void setup(float kp_yaw, float ki_yaw, float kd_yaw);

  float update(IMU& imu, float desiredYaw, float dt);

private:
  PID _pidYaw;
};
