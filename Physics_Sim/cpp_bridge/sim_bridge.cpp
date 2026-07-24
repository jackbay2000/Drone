// sim_bridge.cpp
//
// C API wrapper around Drone_Main's Controller class.
//
// Include path order matters: cpp_bridge/ is searched BEFORE Drone_Main/, so
// our stub headers (Arduino.h, PWMServo.h, IMU.h, FlowSensor.h, Rangefinder.h,
// Position.h) shadow the real Arduino SDK and sensor implementations.  Only
// the pure-logic files from Drone_Main are compiled: PID, AttitudeController,
// YawController, PositionPID, and Controller itself.
//
// Controller no longer sequences waypoints itself -- Drone_Main.ino's loop()
// owns that (see FLIGHT_PATH / _currentWP / _landed / _legYaw there) and just
// calls Controller::update() with the current target + heading each tick.
// This bridge reproduces that exact algorithm against whatever waypoint list
// Python has loaded via sim_add_waypoint(), so tuning behavior (including the
// motor-cut landing behavior and turn-to-face heading) matches what the real
// firmware will do.

#include "sim_bridge.h"

#include <vector>

// Stub headers (found first via -I cpp_bridge)
#include "Arduino.h"
#include "IMU.h"
#include "Position.h"

// Drone_Main control logic (found via -I Drone_Main). controller.h pulls in
// main.h (Gains struct), PWMServo.h, PositionPID.h, AttitudeController.h,
// YawController.h.
#include "controller.h"

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

// Waypoint sequencing -- mirrors Drone_Main.ino::loop() + Waypoints.h::_startLeg
// exactly (same WP_RADIUS, same "arriving at a z==0 waypoint means land" rule,
// same once-per-leg bearing-to-target yaw computation).
struct _Waypoint { float x, y, z; bool keepHeading; };
static std::vector<_Waypoint> _waypoints;
static int   _currentWP = 0;
static bool  _landed    = false;
static float _legYaw    = 0.0f;
static constexpr float WP_RADIUS = 0.15f;   // Drone_Main.ino::WP_RADIUS

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline float _us_to_norm(int us) {
    float n = (us - 1000) / 1000.0f;
    return n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
}

// Mirrors Drone_Main.ino::_startLeg(): bearing from the previous waypoint (or
// takeoff origin) toward FLIGHT_PATH[wpIndex], computed once per leg.
static void _startLeg(int wpIndex) {
    const _Waypoint& wp = _waypoints[wpIndex];
    if (wp.keepHeading) {
        _legYaw = 0.0f;
        return;
    }
    float fromX = (wpIndex == 0) ? 0.0f : _waypoints[wpIndex - 1].x;
    float fromY = (wpIndex == 0) ? 0.0f : _waypoints[wpIndex - 1].y;
    float dx = wp.x - fromX;
    float dy = wp.y - fromY;
    if (dx * dx + dy * dy > 1.0e-6f) _legYaw = atan2f(dy, dx);
    // else: degenerate leg (e.g. pure altitude change) -- keep previous heading
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
    _gains = Gains();          // resets to main.h defaults

    _waypoints.clear();
    _currentWP = 0;
    _landed    = false;
    _legYaw    = 0.0f;

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

void sim_add_waypoint(float x, float y, float z, int keep_heading) {
    _waypoints.push_back({x, y, z, keep_heading != 0});
    if (_waypoints.size() == 1) _startLeg(0);   // first waypoint starts leg 0
}

void sim_clear_waypoints(void) {
    _waypoints.clear();
    _currentWP = 0;
    _landed    = false;
    _legYaw    = 0.0f;
}

int sim_mission_complete(void) {
    return _landed ? 1 : 0;   // mirrors Drone_Main.ino: "complete" == landed
}

int sim_current_waypoint_index(void) {
    return _currentWP;
}

void sim_set_state(float roll, float pitch, float yaw,
                   float x,   float y,     float z,
                   int altitude_stale)
{
    _imu.setRoll(roll);
    _imu.setPitch(pitch);
    _imu.setYaw(yaw);
    _pos.setX(x);
    _pos.setY(y);
    _pos.setZ(z);
    _pos.setAltitudeStale(altitude_stale != 0);
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

    // Waypoint sequencing + landing, mirrors Drone_Main.ino::loop() exactly:
    // check arrival at the CURRENT target, advance/land, then fly the real
    // Drone_Main control loop toward the (possibly just-advanced) target.
    if (!_landed && !_waypoints.empty()) {
        float px = _pos.getX(), py = _pos.getY(), pz = _pos.getZ();
        _Waypoint wp = _waypoints[_currentWP];
        float ex = wp.x - px, ey = wp.y - py, ez = wp.z - pz;
        bool arrived = sqrtf(ex*ex + ey*ey + ez*ez) < WP_RADIUS;

        if (arrived && wp.z == 0.0f) {
            _landed = true;
        } else {
            if (arrived && _currentWP < (int)_waypoints.size() - 1) {
                _currentWP++;
                _startLeg(_currentWP);
            }
            _Waypoint target = _waypoints[_currentWP];
            _ctrl.update(_imu, _pos, target.x, target.y, target.z, _legYaw);
        }
    }

    if (_landed || _waypoints.empty()) {
        _ctrl.halt();
    }

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
