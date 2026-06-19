#pragma once
#include "PID.h"
#include "IMU.h"

struct AttitudeOutput { float roll, pitch; };

class AttitudeController {
public:
  AttitudeController();

  void setup(float kp_roll,  float ki_roll,  float kd_roll,
             float kp_pitch, float ki_pitch, float kd_pitch);

  AttitudeOutput update(IMU& imu, float cvX, float cvY, float dt);

private:
  PID _pidRoll, _pidPitch;
};
