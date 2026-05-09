#include "IMU.h"
#include <math.h>

// Complementary filter weight — how much to trust the gyro vs. accelerometer.
// 0.98 = gyro dominates (fast response), accel slowly corrects drift.
static constexpr float ALPHA = 0.98f;

IMU::IMU() {
  for (int i = 0; i < 3; i++) {
    position[i] = 0.0f;
    velocity[i] = 0.0f;
  }
}

void IMU::setup() {
  Wire.begin();
  Wire.setClock(100000);
  imu.initialize();

  while (!imu.testConnection()) {
    Serial.println("IMU init failed");
    Serial.println(imu.getDeviceID(), HEX);
    Serial.println("Check Setup");
    delay(1000);
  }
  Serial.println("MPU Connection successful");

  // Collect 50 samples to compute biases while stationary
  int16_t ax, ay, az, gx, gy, gz;
  float sum[6] = {0};
  Serial.println("Calibrating — keep IMU still...");
  for (int i = 0; i < 50; i++) {
    imu.getAcceleration(&ax, &ay, &az);
    imu.getRotation(&gx, &gy, &gz);
    sum[0] += ax / 16384.0f;
    sum[1] += ay / 16384.0f;
    sum[2] += az / 16384.0f;
    sum[3] += gx / 131.0f;
    sum[4] += gy / 131.0f;
    sum[5] += gz / 131.0f;
    delay(50);
  }

  float ax_avg = sum[0] / 50.0f;
  float ay_avg = sum[1] / 50.0f;
  float az_avg = sum[2] / 50.0f;

  // Gyro bias is orientation-independent — store directly
  static_accel[3] = sum[3] / 50.0f;
  static_accel[4] = sum[4] / 50.0f;
  static_accel[5] = sum[5] / 50.0f;

  // Derive initial orientation from averaged accelerometer readings.
  // When stationary, the accel vector points opposite to gravity.
  roll  = atan2f(ay_avg, az_avg);
  pitch = atan2f(-ax_avg, sqrtf(ay_avg * ay_avg + az_avg * az_avg));

  // Expected gravity in body frame at this orientation:
  //   g = [-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]
  float g_x =  -sinf(pitch);
  float g_y =   cosf(pitch) * sinf(roll);
  float g_z =   cosf(pitch) * cosf(roll);

  // Pure electronic accel bias = measured average minus the gravity we expect.
  // This residual is orientation-independent and constant across orientations.
  static_accel[0] = ax_avg - g_x;
  static_accel[1] = ay_avg - g_y;
  static_accel[2] = az_avg - g_z;

  Serial.print("Calibration done. Roll: "); Serial.print(degrees(roll), 1);
  Serial.print(" deg  Pitch: "); Serial.print(degrees(pitch), 1);
  Serial.println(" deg");

  lastTime = millis();
}

static constexpr float DEG_TO_RAD_F = 0.01745329251f;  // float version of DEG_TO_RAD

// Orientation correction gate: only correct roll/pitch from accel when linear
// acceleration (gravity-removed) is below this. Keeps accel corruption out of
// the orientation estimate during motion.
static constexpr float ACCEL_VALID_THRESH = 0.10f;  // g's (lin_mag after gravity removal)

// ZUPT: zero velocity when stationary. Keep clearly below ACCEL_VALID_THRESH
// so the two thresholds don't interact.
static constexpr float ZUPT_ACCEL_THRESH = 0.04f;   // g's after gravity removal
static constexpr float ZUPT_GYRO_THRESH  = 0.05f;   // rad/s (~2.9 deg/s)
static constexpr int   ZUPT_MIN_COUNT    = 3;        // consecutive samples (~150ms at 20Hz)
static constexpr float NEAR_STAT_DECAY  = 0.95f;    // velocity decay during low-accel settling

void IMU::update() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);

  // Convert and remove electronic bias (gravity still present here)
  float accel[3] = {
    ax / 16384.0f - static_accel[0],
    ay / 16384.0f - static_accel[1],
    az / 16384.0f - static_accel[2]
  };

  // Debias gyro and convert to rad/s
  float gyro[3] = {
    (gx / 131.0f - static_accel[3]) * DEG_TO_RAD_F,
    (gy / 131.0f - static_accel[4]) * DEG_TO_RAD_F,
    (gz / 131.0f - static_accel[5]) * DEG_TO_RAD
  };

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // Step 1: use the CURRENT (previous-frame) orientation to estimate linear accel.
  // This lets us gate orientation correction before we update roll/pitch.
  float g_x = -sinf(pitch);
  float g_y =  cosf(pitch) * sinf(roll);
  float g_z =  cosf(pitch) * cosf(roll);

  float lin_mag = sqrtf(
    (accel[0]-g_x)*(accel[0]-g_x) +
    (accel[1]-g_y)*(accel[1]-g_y) +
    (accel[2]-g_z)*(accel[2]-g_z)
  );

  // Step 2: update orientation. Only blend in accel-derived angles when linear
  // acceleration is small — otherwise the accel vector isn't pointing along
  // gravity and would corrupt the estimate direction-dependently.
  if (lin_mag < ACCEL_VALID_THRESH) {
    float roll_a  = atan2f(accel[1], accel[2]);
    float pitch_a = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2]));
    roll  = ALPHA * (roll  + gyro[0] * dt) + (1.0f - ALPHA) * roll_a;
    pitch = ALPHA * (pitch + gyro[1] * dt) + (1.0f - ALPHA) * pitch_a;
  } else {
    roll  += gyro[0] * dt;
    pitch += gyro[1] * dt;
  }

  // Step 3: recompute gravity with the updated orientation and remove it.
  g_x = -sinf(pitch);
  g_y =  cosf(pitch) * sinf(roll);
  g_z =  cosf(pitch) * cosf(roll);

  accel[0] -= g_x;
  accel[1] -= g_y;
  accel[2] -= g_z;

  // Step 4: ZUPT — zero velocity when stationary.
  float accel_mag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
  float gyro_mag  = sqrtf(gyro[0]*gyro[0]   + gyro[1]*gyro[1]   + gyro[2]*gyro[2]);

  if ((accel_mag < ZUPT_ACCEL_THRESH) && (gyro_mag < ZUPT_GYRO_THRESH))
    zupt_count = min(zupt_count + 1, ZUPT_MIN_COUNT + 1);
  else
    zupt_count = 0;

  // Bleed residual velocity during the sensor settling window after motion stops.
  // Only active when accel_mag is low — has no effect during real motion.
  if (accel_mag < ACCEL_VALID_THRESH)
    for (int i = 0; i < 3; i++)
      velocity[i] *= NEAR_STAT_DECAY;

  for (int i = 0; i < 3; i++)
    velocity[i] += accel[i] * dt;

  if (zupt_count >= ZUPT_MIN_COUNT)
    velocity[0] = velocity[1] = velocity[2] = 0.0f;

  for (int i = 0; i < 3; i++)
    position[i] += velocity[i] * dt;
}

float IMU::getX() const { return position[0]; }
float IMU::getY() const { return position[1]; }
float IMU::getZ() const { return position[2]; }

void IMU::reset() {
  for (int i = 0; i < 3; i++) {
    position[i] = 0.0f;
    velocity[i] = 0.0f;
  }
}
