#pragma once

struct Gains {
  // Attitude — tune first, props off
  float kp_roll  = 0, ki_roll  = 0, kd_roll  = 0;
  float kp_pitch = 0, ki_pitch = 0, kd_pitch = 0;
  float kp_yaw   = 0, ki_yaw   = 0, kd_yaw   = 0;
  // Position — tune only after attitude is solid
  float kp_x = 0, ki_x = 0, kd_x = 0;
  float kp_y = 0, ki_y = 0, kd_y = 0;
  float kp_z = 0, ki_z = 0, kd_z = 0;
};


Gains loadGains();
void  printGains(const Gains& g);
