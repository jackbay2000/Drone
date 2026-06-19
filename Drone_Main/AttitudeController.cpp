#include "AttitudeController.h"

AttitudeController::AttitudeController()
  : _pidRoll (0, 0, 0, -0.5f, 0.5f)
  , _pidPitch(0, 0, 0, -0.5f, 0.5f) {}

void AttitudeController::setup(float kp_roll,  float ki_roll,  float kd_roll,
                                float kp_pitch, float ki_pitch, float kd_pitch) {
  _pidRoll .setGains(kp_roll,  ki_roll,  kd_roll);
  _pidPitch.setGains(kp_pitch, ki_pitch, kd_pitch);
}

AttitudeOutput AttitudeController::update(IMU& imu, float cvX, float cvY, float dt) {
  float desiredPitch = cvX;
  float desiredRoll  = -cvY;

  AttitudeOutput out;
  out.roll  = _pidRoll .compute(desiredRoll  - imu.getRoll(),  imu.getRoll(),  dt);
  out.pitch = _pidPitch.compute(desiredPitch - imu.getPitch(), imu.getPitch(), dt);
  return out;
}
