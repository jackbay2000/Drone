#include "main.h"
#include "Waypoints.h"

IMU         imu;
FlowSensor  flow(10);
Rangefinder rangefinder;
Position    position;
Controller  controller;
Gains       gains;

// ── Kill switch ──────────────────────────────────────────────────────────────
static constexpr uint8_t KILL_GND_PIN = 6;
static constexpr uint8_t KILL_SIG_PIN = 7;

enum class DroneState : uint8_t { RUNNING, HALTED, RESETTING };
static DroneState _state;

// ── Mission state ────────────────────────────────────────────────────────────
// FLIGHT_PATH / NUM_WAYPOINTS / WP_RADIUS come from Waypoints.h -- that's the
// only file you need to edit to change the mission.
static int   _currentWP = 0;
static bool  _landed    = false;
static float _legYaw    = 0.0f;   // heading target for the current leg

// ── Init ─────────────────────────────────────────────────────────────────────

// Sets _legYaw for the leg ending at FLIGHT_PATH[wpIndex]: locked to 0 if
// that waypoint wants keepHeading, otherwise the bearing from the previous
// waypoint (or takeoff origin) toward it -- computed once per leg so the
// drone has the whole leg to turn, and doesn't jitter as it nears arrival.
static void _startLeg(int wpIndex) {
  const Waypoint& wp = FLIGHT_PATH[wpIndex];
  if (wp.keepHeading) {
    _legYaw = 0.0f;
    return;
  }
  float fromX = (wpIndex == 0) ? 0.0f : FLIGHT_PATH[wpIndex - 1].x;
  float fromY = (wpIndex == 0) ? 0.0f : FLIGHT_PATH[wpIndex - 1].y;
  float dx = wp.x - fromX;
  float dy = wp.y - fromY;
  if (dx * dx + dy * dy > 1.0e-6f) _legYaw = atan2f(dy, dx);
  // else: degenerate leg (e.g. pure altitude change) -- keep previous heading
}

static void doInit() {
  imu.setup();
  rangefinder.setup();
  flow.setup();
  position.reset();
  position.setup(imu, flow, rangefinder);
  _currentWP = 0;
  _landed    = false;
  _startLeg(_currentWP);
  controller.setup(gains);
}

void setup() {
  Serial.begin(115200);

  pinMode(KILL_GND_PIN, OUTPUT);
  digitalWrite(KILL_GND_PIN, LOW);
  pinMode(KILL_SIG_PIN, INPUT_PULLUP);

  doInit();
  _state = DroneState::RUNNING;
}

// ── Telemetry ────────────────────────────────────────────────────────────────

static void printTelemetry() {
  static unsigned long lastPrint = 0;
  if (micros() - lastPrint < 100000UL) return;
  lastPrint = micros();

  Serial.print("X: ");      Serial.print(position.getX(), 3);
  Serial.print("  Y: ");    Serial.print(position.getY(), 3);
  Serial.print("  Z: ");    Serial.print(position.getZ(), 3);
  Serial.print("  Roll: "); Serial.print(degrees(imu.getRoll()),  1);
  Serial.print("  Pitch: ");Serial.print(degrees(imu.getPitch()), 1);
  Serial.print("  Yaw: ");  Serial.print(degrees(imu.getYaw()),   1);
  Serial.print("  WP: ");   Serial.println(_currentWP);
}

// ── Main loop ────────────────────────────────────────────────────────────────

void loop() {
  bool switchClosed = (digitalRead(KILL_SIG_PIN) == LOW);

  if (switchClosed && _state == DroneState::RUNNING) {
    controller.halt();
    _state = DroneState::HALTED;
    return;
  }

  if (_state == DroneState::HALTED) {
    if (!switchClosed) {
      _state = DroneState::RESETTING;
      doInit();
      delay(10000);
      _state = DroneState::RUNNING;
    }
    return;
  }

  // Timing gate — 250 Hz
  static unsigned long lastLoop = 0;
  static constexpr unsigned long LOOP_PERIOD_US = 4000;
  unsigned long now = micros();
  if (now - lastLoop < LOOP_PERIOD_US) return;
  lastLoop = now;

  imu.update();
  position.update(imu, flow, rangefinder);

  if (_landed) {
    controller.halt();
    printTelemetry();
    return;
  }

  // Waypoint sequencing
  float px = position.getX();
  float py = position.getY();
  float pz = position.getZ();

  const Waypoint& wp = FLIGHT_PATH[_currentWP];
  float ex = wp.x - px;
  float ey = wp.y - py;
  float ez = wp.z - pz;

  if (sqrtf(ex*ex + ey*ey + ez*ez) < WP_RADIUS) {
    if (wp.z == 0.0f) {
      _landed = true;
      controller.halt();
      printTelemetry();
      return;
    }
    if (_currentWP < NUM_WAYPOINTS - 1) {
      _currentWP++;
      _startLeg(_currentWP);
    }
  }

  controller.update(imu, position,
                    FLIGHT_PATH[_currentWP].x,
                    FLIGHT_PATH[_currentWP].y,
                    FLIGHT_PATH[_currentWP].z,
                    _legYaw);
  printTelemetry();
}
