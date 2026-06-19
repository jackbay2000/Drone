#pragma once
#include <Arduino.h>

// ── Gains — edit these and recompile ─────────────────────────────────────────
struct Gains {
  float kp_roll  = 3.5f,  ki_roll  = 0.04f,  kd_roll  = 0.15f;
  float kp_pitch = 3.5f,  ki_pitch = 0.04f,  kd_pitch = 0.15f;
  float kp_yaw   = 2.0f,  ki_yaw   = 0.02f,  kd_yaw   = 0.08f;
  float kp_x = 0.25f,  ki_x = 0.005f,  kd_x = 0.15f;
  float kp_y = 0.25f,  ki_y = 0.005f,  kd_y = 0.15f;
  float kp_z = 0.40f,  ki_z = 0.02f,   kd_z = 0.20f;
};

#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"
#include "Position.h"
#include "controller.h"
