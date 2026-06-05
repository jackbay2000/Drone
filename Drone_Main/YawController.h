#pragma once
#include "PID.h"
#include "IMU.h"

class YawController {
public:
  YawController();

  void setup(float kp_yaw, float ki_yaw, float kd_yaw);

  // desiredYaw : absolute heading in radians (from ControlVector.yaw)
  // Returns a normalized yaw moment in [-0.5, 0.5].
  //
  // How it works: the two CCW propellers (M1 FL, M4 RR) and two CW propellers
  // (M2 FR, M3 RL) produce opposing aerodynamic drag torques that cancel at
  // equal throttle.  Speeding up the CCW props relative to the CW props creates
  // a net CCW aerodynamic torque, and the frame reacts by rotating CW.
  // Therefore: positive output → CCW props faster → frame rotates CW.
  float update(IMU& imu, float desiredYaw, float dt);

  void reset();

private:
  PID _pidYaw;
};
