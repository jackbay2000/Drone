// Position_Test — standalone bench test for orientation (IMU) and linear
// position (optical flow + rangefinder) tracking, with NO motors and NO
// controller loop. Uses the exact same IMU/Position/FlowSensor/Rangefinder
// source files as Drone_Main (copied verbatim, not reimplemented), so
// whatever this prints is exactly what Controller would see on the real
// drone -- this is for verifying the sensors/estimator alone are sane
// (mounting, wiring, calibration, drift, yaw-frame tracking) before ever
// touching props.
//
// Wiring matches Drone_Main.ino: MPU6050 on I2C, PMW3901 CS on pin 10,
// VL53L1X on I2C.

#include "IMU.h"
#include "FlowSensor.h"
#include "Rangefinder.h"
#include "Position.h"

IMU         imu;
FlowSensor  flow(10);
Rangefinder rangefinder;
Position    position;

// CSV telemetry -- pipe this straight into a .csv file with
// Position_Test/log_serial.py (or any serial logger) instead of hand-copying
// from the Serial Monitor. Header is printed once at boot; every line after
// that is pure comma-separated data, no labels, so it's directly loadable
// with pandas.read_csv() / Excel with no cleanup.
static constexpr char CSV_HEADER[] = "t_ms,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg";

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  imu.setup();
  rangefinder.setup();
  flow.setup();
  position.reset();
  position.setup(imu, flow, rangefinder);

  Serial.println("# Position_Test ready.");
  Serial.println(CSV_HEADER);
}

void loop() {
  // Timing gate — 250 Hz, matches Drone_Main.ino's main loop exactly, since
  // IMU's complementary filter and Position's optical-flow integration are
  // both tuned assuming that update rate.
  static unsigned long lastLoop = 0;
  static constexpr unsigned long LOOP_PERIOD_US = 4000;
  unsigned long now = micros();
  if (now - lastLoop < LOOP_PERIOD_US) return;
  lastLoop = now;

  imu.update();
  position.update(imu, flow, rangefinder);

  // Telemetry — 10 Hz, matches Drone_Main.ino's printTelemetry() rate.
  static unsigned long lastPrint = 0;
  if (micros() - lastPrint < 100000UL) return;
  lastPrint = micros();

  Serial.print(millis());                          Serial.print(',');
  Serial.print(position.getX(), 4);                 Serial.print(',');
  Serial.print(position.getY(), 4);                 Serial.print(',');
  Serial.print(position.getZ(), 4);                 Serial.print(',');
  Serial.print(degrees(imu.getRoll()),  3);          Serial.print(',');
  Serial.print(degrees(imu.getPitch()), 3);          Serial.print(',');
  Serial.println(degrees(imu.getYaw()), 3);
}
