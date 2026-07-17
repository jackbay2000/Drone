#pragma once
#include <Arduino.h>

// ── Gains — edit these and recompile ─────────────────────────────────────────
// Found by Physics_Sim/tools/tune_gains.py (Monte Carlo search, 500 trials)
// against the component_list.json vehicle (698g) flying Waypoints.h's mission.
// Sim result: no divergence, mission completed in 28.1s, roll RMS 2.4deg,
// pitch RMS 1.7deg, altitude RMS error 13.5cm. See PR/conversation for the
// full report before trusting this on real hardware -- untested on real
// sensors (noise/latency/vibration aren't modeled in the sim).
struct Gains {
  float kp_roll  = 1.9586f,  ki_roll  = 0.2372f,  kd_roll  = 0.2796f;
  float kp_pitch = 0.6890f,  ki_pitch = 0.0445f,  kd_pitch = 0.1042f;
  float kp_yaw   = 1.8905f,  ki_yaw   = 0.0968f,  kd_yaw   = 0.3508f;
  float kp_x = 0.0541f,  ki_x = 0.0360f,  kd_x = 0.6000f;
  float kp_y = 0.2893f,  ki_y = 0.0188f,  kd_y = 0.5932f;
  float kp_z = 1.5000f,  ki_z = 0.0145f,  kd_z = 0.2691f;
};

#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"
#include "Position.h"
#include "controller.h"
