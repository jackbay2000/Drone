#include "main.h"

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

// ── Waypoints ────────────────────────────────────────────────────────────────
struct Waypoint { float x, y, z; };

static const Waypoint FLIGHT_PATH[] = {
  { 0.0f,  0.0f,  1.0f},   // takeoff: rise 1 m
  { 2.0f,  0.0f,  1.0f},   // move 2 m forward
  { 2.0f,  1.5f,  1.0f},   // move 1.5 m left
  { 0.0f,  0.0f,  1.0f},   // return overhead
  { 0.0f,  0.0f,  0.0f},   // land
};
static constexpr int   NUM_WAYPOINTS = sizeof(FLIGHT_PATH) / sizeof(FLIGHT_PATH[0]);
static constexpr float WP_RADIUS     = 0.15f;

static int  _currentWP = 0;
static bool _landed    = false;

// ── Init ─────────────────────────────────────────────────────────────────────

static void doInit() {
  imu.setup();
  rangefinder.setup();
  flow.setup();
  position.reset();
  position.setup(imu, flow, rangefinder);
  _currentWP = 0;
  _landed    = false;
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
    }
  }

  controller.update(imu, position,
                    FLIGHT_PATH[_currentWP].x,
                    FLIGHT_PATH[_currentWP].y,
                    FLIGHT_PATH[_currentWP].z);
  printTelemetry();
}
