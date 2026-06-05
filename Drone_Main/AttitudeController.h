#pragma once
#include "PID.h"
#include "IMU.h"

// Normalized roll/pitch motor deltas in the range [-0.5, 0.5]
struct AttitudeOutput {
  float roll;
  float pitch;
};

class AttitudeController {
public:
  AttitudeController();

  void setup(float kp_roll,  float ki_roll,  float kd_roll,
             float kp_pitch, float ki_pitch, float kd_pitch);

  // cvX : forward motion command from PositionPID — maps directly to desired pitch
  // cvY : leftward motion command from PositionPID — inverted to derive desired roll
  //       (positive left demand = left side down = negative roll in this convention)
  // Returns normalized roll/pitch deltas ready for motor mixing.
  AttitudeOutput update(IMU& imu, float cvX, float cvY, float dt);

  void reset();

private:
  PID _pidRoll;
  PID _pidPitch;
};
