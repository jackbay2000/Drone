#include <Arduino.h>
#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"
#include "Position.h"
#include "controller.h"
#include "Gains.h"

IMU         imu;
FlowSensor  flow(10);    // CS pin 10 — MOSI=11, MISO=12, SCK=13
Rangefinder rangefinder;
Position    position;
Controller  controller;

// ── Kill switch ───────────────────────────────────────────────────────────────
// Wire a normally-open switch between Pin 6 and Pin 7 on the Teensy 4.1.
// All three pins (6, 7, 8) are adjacent — use them as a 3-pin Dupont header:
//   Pin 6: OUTPUT LOW  (software GND for the switch)
//   Pin 7: INPUT_PULLUP (signal — reads LOW when switch is closed)
//   Pin 8: NC (not driven; physically fills the header so either end can be GND)
static constexpr uint8_t KILL_GND_PIN = 6;
static constexpr uint8_t KILL_SIG_PIN = 7;

enum class DroneState : uint8_t { RUNNING, HALTED, RESETTING };
static DroneState _state;
static Gains      _savedGains;   // loaded once at power-on; survives all resets

// ── Waypoints ─────────────────────────────────────────────────────────────────
// Edit your flight path here. Re-applied on every reset so the drone starts
// fresh from its new ground position each time.
static void addWaypoints() {
  controller.clearWaypoints();
  controller.addWaypoint( 0.0f,  0.0f,  1.0f);   // takeoff: rise 1 m
  controller.addWaypoint( 2.0f,  0.0f,  1.0f);   // move 2 m forward
  controller.addWaypoint( 2.0f,  1.5f,  1.0f);   // move 1.5 m left
  controller.addWaypoint( 0.0f,  0.0f,  1.0f);   // return overhead
  controller.addWaypoint( 0.0f,  0.0f,  0.0f);   // land
}

// Re-inits all sensors, zeroes position, re-arms ESCs.
// Does NOT reload gains — those stay in _savedGains from power-on.
static void doInit() {
  imu.setup();             // Wire.begin() + MPU6050 calibration (~2.5 s)
  rangefinder.setup();     // VL53L1X on I2C  (addr 0x29)
  flow.setup();            // PMW3901 on SPI  (CS pin 10)
  position.reset();        // zero accumulated X/Y
  position.setup(imu, flow, rangefinder);
  addWaypoints();
  controller.setup(_savedGains); // attaches ESCs, 2 s disarm signal, applies gains
}

void setup() {
  Serial.begin(115200);
  _savedGains = loadGains();   // 8-second paste window — runs only at power-on

  pinMode(KILL_GND_PIN, OUTPUT);
  digitalWrite(KILL_GND_PIN, LOW);
  pinMode(KILL_SIG_PIN, INPUT_PULLUP);

  doInit();
  _state = DroneState::RUNNING;
}

// Print position and orientation at 10 Hz without blocking the control loop
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
  Serial.print("  [gX: ");  Serial.print(imu.rawGX(), 1);
  Serial.print(" gY: ");    Serial.print(imu.rawGY(), 1);
  Serial.print(" gZ: ");    Serial.print(imu.rawGZ(), 1);
  Serial.print(" | aX: ");  Serial.print(imu.rawAX(), 2);
  Serial.print(" aY: ");    Serial.print(imu.rawAY(), 2);
  Serial.print(" aZ: ");    Serial.print(imu.rawAZ(), 2);
  Serial.println("]");
  //Serial.print("  WP: ");   Serial.println(controller.isComplete() ? "DONE" : "flying");
}

void loop() {
  bool switchClosed = (digitalRead(KILL_SIG_PIN) == LOW);

  // ── Transition: RUNNING → HALTED ──────────────────────────────────────────
  if (switchClosed && _state == DroneState::RUNNING) {
    controller.halt();
    _state = DroneState::HALTED;
    Serial.println("Kill switch: motors halted.");
    return;
  }

  // ── Transition: HALTED → RESETTING → RUNNING ──────────────────────────────
  if (_state == DroneState::HALTED) {
    if (!switchClosed) {
      _state = DroneState::RESETTING;
      Serial.println("Kill switch: re-initialising...");
      doInit();   // sensor cal + ESC re-arm in 4.5 seconds
      Serial.println("Kill switch: waiting 10 s before enabling motors...");
      delay(10000);
      _state = DroneState::RUNNING;
      Serial.println("Kill switch: running.");
    }
    return;
  }

  // ── Normal flight ─────────────────────────────────────────────────────────
  imu.update();
  position.update(imu, flow, rangefinder);
  controller.update(imu, position);
  printTelemetry();
}
