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
  // Forward command becomes desired pitch — a positive pitch tilts the nose down,
  // generating thrust in the forward direction.
  // Left command is inverted for roll — rolling left (negative roll) shifts lift
  // leftward and moves the drone in the +Y direction.
  float desiredPitch = cvX;
  float desiredRoll  = -cvY;

  float rollErr  = desiredRoll  - imu.getRoll();
  float pitchErr = desiredPitch - imu.getPitch();

  AttitudeOutput out;
  out.roll  = _pidRoll .compute(rollErr,  imu.getRoll(),  dt);
  out.pitch = _pidPitch.compute(pitchErr, imu.getPitch(), dt);
  return out;
}

void AttitudeController::reset() {
  _pidRoll .reset();
  _pidPitch.reset();
}
