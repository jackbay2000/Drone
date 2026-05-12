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

void setup() {
  Serial.begin(115200);
  Gains gains = loadGains();

  // ── Waypoints ──────────────────────────────────────────────────────────────
  // Define your flight path here in metres, relative to the start position.
  // The drone will visit each point in order then hold position at the last one.
  // Format: addWaypoint(x_forward, y_left, z_up)
  //
  // Example: rise 1 m, move 2 m forward, then 1.5 m left, then return overhead
  controller.addWaypoint( 0.0f,  0.0f,  1.0f);   // takeoff: rise 1 m
  controller.addWaypoint( 2.0f,  0.0f,  1.0f);   // move 2 m forward
  controller.addWaypoint( 2.0f,  1.5f,  1.0f);   // move 1.5 m left
  controller.addWaypoint( 0.0f,  0.0f,  1.0f);   // return overhead
  controller.addWaypoint( 0.0f,  0.0f,  0.0f);   // land

  // ── Sensor init ────────────────────────────────────────────────────────────
  imu.setup();             // Wire.begin() + MPU6050 calibration (~2.5 s)
  rangefinder.setup();     // VL53L1X on I2C  (addr 0x29)
  flow.setup();            // PMW3901 on SPI  (CS pin 10)
  position.setup(imu, flow, rangefinder);

  // ── Controller init ────────────────────────────────────────────────────────
  controller.setup(gains); // attaches ESCs, sends 2 s disarm signal, applies gains
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
  Serial.print("  [gX: "); Serial.print(imu.rawGX(), 1);
  Serial.print(" gY: ");   Serial.print(imu.rawGY(), 1);
  Serial.print(" gZ: ");   Serial.print(imu.rawGZ(), 1);
  Serial.print(" | aX: "); Serial.print(imu.rawAX(), 2);
  Serial.print(" aY: ");   Serial.print(imu.rawAY(), 2);
  Serial.print(" aZ: ");   Serial.print(imu.rawAZ(), 2);
  Serial.println("]");
  //Serial.print("  WP: ");   Serial.println(controller.isComplete() ? "DONE" : "flying");
}

void loop() {
  imu.update();
  position.update(imu, flow, rangefinder);
  controller.update(imu, position);
  printTelemetry();
}
