#pragma once

#ifdef _WIN32
#  define SIM_API __declspec(dllexport)
#else
#  define SIM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Reset everything: zero time, clear ESC state, destroy old Controller.
// Call once before setting up a new scenario.
SIM_API void sim_init(void);

// Apply gains to the Controller.  Triggers a lazy re-setup on the next
// sim_update() call so the new gains take effect immediately.
SIM_API void sim_set_gains(
    float kp_roll,  float ki_roll,  float kd_roll,
    float kp_pitch, float ki_pitch, float kd_pitch,
    float kp_yaw,   float ki_yaw,   float kd_yaw,
    float kp_x,     float ki_x,     float kd_x,
    float kp_y,     float ki_y,     float kd_y,
    float kp_z,     float ki_z,     float kd_z
);

// ---------------------------------------------------------------------------
// Waypoints  (mirrors Drone_Main.ino's FLIGHT_PATH / _currentWP / _landed)
// ---------------------------------------------------------------------------
// keep_heading: 1 = nose locked at takeoff heading (0 rad) for this leg,
//               0 = nose turns to face this waypoint. A waypoint with z == 0
//               is a landing: arrival cuts the motors and ends the mission.
SIM_API void sim_add_waypoint(float x, float y, float z, int keep_heading);
SIM_API void sim_clear_waypoints(void);
SIM_API int  sim_mission_complete(void);   // 1 = done, 0 = flying

// ---------------------------------------------------------------------------
// Per-step interface
// ---------------------------------------------------------------------------

// Inject sensor state (roll/pitch/yaw in radians; x/y/z in metres).
// Call before sim_update() each step.
SIM_API void sim_set_state(float roll, float pitch, float yaw,
                           float x,   float y,     float z);

// Advance the simulated clock by dt_us microseconds, then run one
// Controller::update() tick.  motor_out[4] receives normalized [0,1]
// motor commands in sim order: [FL-CCW, FR-CW, RR-CCW, RL-CW].
SIM_API void sim_update(unsigned long dt_us, float motor_out[4]);

#ifdef __cplusplus
}
#endif
