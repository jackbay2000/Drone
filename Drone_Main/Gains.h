#pragma once

// Starting gain estimates for a slow indoor quadcopter.
// Tune in this order — always with propellers off first for attitude:
//   1. kp_roll/pitch until the drone holds level but just starts to oscillate
//   2. kd_roll/pitch to damp the oscillation
//   3. Small ki_roll/pitch to remove steady-state lean
//   4. Repeat steps 1-3 for yaw
//   5. Tune kp_z, then kd_z, then ki_z for altitude hold
//   6. Tune X/Y position gains last — attitude must be solid first
struct Gains {
  // ── Attitude — tune first, props off ─────────────────────────────────────
  // Error in radians, output normalized [-0.5, 0.5].
  // kp drives a correction proportional to angle error;
  // kd damps oscillation; ki removes steady-state lean.
  float kp_roll  = 3.5f,  ki_roll  = 0.04f,  kd_roll  = 0.15f;
  float kp_pitch = 3.5f,  ki_pitch = 0.04f,  kd_pitch = 0.15f;

  // ── Yaw — tune after roll/pitch ───────────────────────────────────────────
  // Yaw responds slower than roll/pitch — keep kp lower to avoid overshoot.
  float kp_yaw   = 2.0f,  ki_yaw   = 0.02f,  kd_yaw   = 0.08f;

  // ── Position — tune only after attitude is solid ──────────────────────────
  // Error in meters, output in radians (capped at MAX_TILT = 0.35 rad).
  // A kp of 0.25 maps a 1 m error to a 0.25 rad tilt — gentle for indoor flight.
  // Derivative helps damp position overshoot at waypoint arrivals.
  float kp_x = 0.25f,  ki_x = 0.005f,  kd_x = 0.15f;
  float kp_y = 0.25f,  ki_y = 0.005f,  kd_y = 0.15f;

  // ── Altitude — tune before X/Y ────────────────────────────────────────────
  // Error in meters, output normalized throttle delta [-0.4, 0.4].
  float kp_z = 0.40f,  ki_z = 0.02f,  kd_z = 0.20f;
};

Gains loadGains();
void  printGains(const Gains& g);
