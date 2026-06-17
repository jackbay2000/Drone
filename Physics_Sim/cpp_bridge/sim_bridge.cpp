// sim_bridge.cpp
//
// C API wrapper around Drone_Main's Controller class.
//
// Include path order matters: cpp_bridge/ is searched BEFORE Drone_Main/, so
// our stub headers (Arduino.h, Servo.h, IMU.h, Position.h) shadow the real
// Arduino SDK and sensor implementations.  Only the pure-logic files from
// Drone_Main are compiled: PID, AttitudeController, YawController,
// PositionPID, and Controller itself.

#include "sim_bridge.h"

// Stub headers (found first via -I cpp_bridge)
#include "Arduino.h"
#include "Servo.h"
#include "IMU.h"
#include "Position.h"

// Drone_Main control logic headers (found via -I Drone_Main)
#include "controller.h"
#include "Gains.h"

// ---------------------------------------------------------------------------
// Global storage for extern symbols declared in stub headers
// ---------------------------------------------------------------------------
unsigned long _sim_micros = 0;
_SerialStub   Serial;
int _servo_values[MAX_SERVOS] = {};
int _servo_count = 0;

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------
static Controller  _ctrl;
static IMU         _imu;
static Position    _pos;
static FlowSensor  _flow(0);
static Rangefinder _range;
static Gains       _gains;

static bool _initialized = false;   // true after first setup() call

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline float _us_to_norm(int us) {
    float n = (us - 1000) / 1000.0f;
    return n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
}

// ---------------------------------------------------------------------------
// C API implementation
// ---------------------------------------------------------------------------
extern "C" {

void sim_init(void) {
    _sim_micros   = 0;
    _servo_count  = 0;
    for (int i = 0; i < MAX_SERVOS; i++) _servo_values[i] = 1000;

    _ctrl  = Controller();
    _imu   = IMU();
    _pos   = Position();
    _gains = Gains();          // resets to Gains.h defaults

    _initialized = false;
}

void sim_set_gains(
    float kp_roll,  float ki_roll,  float kd_roll,
    float kp_pitch, float ki_pitch, float kd_pitch,
    float kp_yaw,   float ki_yaw,   float kd_yaw,
    float kp_x,     float ki_x,     float kd_x,
    float kp_y,     float ki_y,     float kd_y,
    float kp_z,     float ki_z,     float kd_z)
{
    _gains.kp_roll  = kp_roll;   _gains.ki_roll  = ki_roll;   _gains.kd_roll  = kd_roll;
    _gains.kp_pitch = kp_pitch;  _gains.ki_pitch = ki_pitch;  _gains.kd_pitch = kd_pitch;
    _gains.kp_yaw   = kp_yaw;    _gains.ki_yaw   = ki_yaw;    _gains.kd_yaw   = kd_yaw;
    _gains.kp_x     = kp_x;      _gains.ki_x     = ki_x;      _gains.kd_x     = kd_x;
    _gains.kp_y     = kp_y;      _gains.ki_y     = ki_y;      _gains.kd_y     = kd_y;
    _gains.kp_z     = kp_z;      _gains.ki_z     = ki_z;      _gains.kd_z     = kd_z;
    _initialized = false;   // force re-setup with new gains
}

void sim_add_waypoint(float x, float y, float z) {
    _ctrl.addWaypoint(x, y, z);
}

void sim_clear_waypoints(void) {
    _ctrl.clearWaypoints();
}

int sim_mission_complete(void) {
    return _ctrl.isComplete() ? 1 : 0;
}

void sim_set_state(float roll, float pitch, float yaw,
                   float x,   float y,     float z)
{
    _imu.setRoll(roll);
    _imu.setPitch(pitch);
    _imu.setYaw(yaw);
    _pos.setX(x);
    _pos.setY(y);
    _pos.setZ(z);
}

void sim_update(unsigned long dt_us, float motor_out[4]) {
    // Lazy setup: called on first update and whenever gains change
    if (!_initialized) {
        _servo_count = 0;          // reset so attach() assigns indices 0-3 fresh
        _ctrl.setup(_gains);       // delay(2000) is a no-op in our stub
        _initialized = true;
    }

    // Advance simulated clock — controller.cpp reads micros() to compute its
    // own dt, so this is the only clock tick it sees.
    _sim_micros += dt_us;

    // Reset ESC values each tick so a safety-cut (all 1000 µs) is visible
    for (int i = 0; i < MAX_SERVOS; i++) _servo_values[i] = 1000;

    // Run the real Drone_Main control loop
    _ctrl.update(_imu, _pos);

    // Controller::setup() calls attach() in order: M1=FL, M2=FR, M3=RL, M4=RR
    // → servo indices:                              0     1     2     3
    // Sim motor order expected by PhysicsEngine:   FL,   FR,   RR,   RL
    //                                               0     1     2     3
    // So: sim[0]=idx0(M1), sim[1]=idx1(M2), sim[2]=idx3(M4), sim[3]=idx2(M3)
    motor_out[0] = _us_to_norm(_servo_values[0]);   // FL  (M1)
    motor_out[1] = _us_to_norm(_servo_values[1]);   // FR  (M2)
    motor_out[2] = _us_to_norm(_servo_values[3]);   // RR  (M4)
    motor_out[3] = _us_to_norm(_servo_values[2]);   // RL  (M3)
}

} // extern "C"
