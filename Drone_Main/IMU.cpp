#include "IMU.h"
#include <math.h>

static constexpr float FILTER_TAU  = 0.75f;
static constexpr float RAD_PER_DEG = 0.01745329251f;

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Axis remapping — IMU is rotated 90° relative to the optical flow sensor.
static constexpr float FLOW_X_SIGN =  1.0f;   // flip to -1 if pitch is backwards
static constexpr float FLOW_Y_SIGN = -1.0f;   // flip to +1 if roll  is backwards

// Rate at which the Z-axis gyro bias estimate is updated in-flight.
// While the yaw rate stays near zero, the bias slowly tracks thermal drift.
// Lower = more conservative (less risk of absorbing real slow rotation).
// At 0.0001 and ~250 Hz the bias converges in roughly 40 s of stationary hover.
static constexpr float YAW_BIAS_LEARN = 0.0001f;

IMU::IMU() {}

void IMU::setup() {
  Wire.begin();
  Wire.setClock(400000);

  _imu.initialize();
  while (!_imu.testConnection()) {
    Serial.println("IMU not found — check wiring");
    delay(500);
  }
  Serial.println("IMU connected");

  Wire.beginTransmission(0x68);
  Wire.write(0x6C);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

//Gyro check
  {
    bool gx_alive = false, gy_alive = false, gz_alive = false;
    for (int i = 0; i < 20; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      _imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      if (gx != 0) gx_alive = true;
      if (gy != 0) gy_alive = true;
      if (gz != 0) gz_alive = true;
      delay(5);
    }
    Serial.print(F("Gyro axes: X="));  Serial.print(gx_alive ? F("OK") : F("DEAD"));
    Serial.print(F("  Y="));           Serial.print(gy_alive ? F("OK") : F("DEAD"));
    Serial.print(F("  Z="));           Serial.println(gz_alive ? F("OK") : F("DEAD — replace MPU6050, yaw will not work"));
  }

// about 5 seconds of sample to grab data
  const int N = 1000;
  int16_t ax, ay, az, gx, gy, gz;
  double sa[3] = {}, sg[3] = {};

  Serial.println("Calibrating — keep still for ~5 s...");
  for (int i = 0; i < N; i++) {
    _imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sa[0] += ax / 16384.0;  sa[1] += ay / 16384.0;  sa[2] += az / 16384.0;
    sg[0] += gx / 131.0;    sg[1] += gy / 131.0;    sg[2] += gz / 131.0;
    delay(5);
  }

  // Store biases in flow-sensor frame (X-Y swap, Z unchanged)
  // _gyroBias[0] = flow GX bias = IMU GY bias  (sg[1])
  // _gyroBias[1] = flow GY bias = IMU GX bias  (sg[0])
  // _gyroBias[2] = flow GZ bias = IMU GZ bias  (sg[2])
  _gyroBias[0] = sg[1] / N;
  _gyroBias[1] = sg[0] / N;
  _gyroBias[2] = sg[2] / N;

  // Seed the filter using remapped accel averages
  float ax_avg = FLOW_X_SIGN * (float)(sa[1] / N);  // flow X = IMU Y
  float ay_avg = FLOW_Y_SIGN * (float)(sa[0] / N);  // flow Y = IMU X
  float az_avg =               (float)(sa[2] / N);  // flow Z = IMU Z

  _rollOffset  = atan2f(ay_avg, az_avg);
  _pitchOffset = atan2f(-ax_avg, sqrtf(ay_avg * ay_avg + az_avg * az_avg));
  _roll  = 0.0f;
  _pitch = 0.0f;
  _yaw   = 0.0f;

  Serial.print("Calibration done.  Mounting offset — Roll: ");  Serial.print(degrees(_rollOffset),  1);
  Serial.print(" deg   Pitch: "); Serial.print(degrees(_pitchOffset), 1); Serial.println(" deg");

  _lastMicros = micros();
}

void IMU::update() {
  int16_t ax, ay, az, gx, gy, gz;
  _imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1.0e-6f;
  _lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  // Raw values before any remapping — used to identify the physical vertical axis.
  // At rest: the accel axis reading about 1.0 g is vertical (= gravity direction).
  _rawIMUG[0] = gx / 131.0f;    _rawIMUA[0] = ax / 16384.0f;
  _rawIMUG[1] = gy / 131.0f;    _rawIMUA[1] = ay / 16384.0f;
  _rawIMUG[2] = gz / 131.0f;    _rawIMUA[2] = az / 16384.0f;

  // Remap to flow-sensor frame: flow X = IMU Y, flow Y = IMU X, flow Z = IMU Z
  float a_x = FLOW_X_SIGN * (ay / 16384.0f);
  float a_y = FLOW_Y_SIGN * (ax / 16384.0f);
  float a_z =               (az / 16384.0f);

  _gRate[0] = FLOW_X_SIGN * (gy / 131.0f - _gyroBias[0]) * RAD_PER_DEG;
  _gRate[1] = FLOW_Y_SIGN * (gx / 131.0f - _gyroBias[1]) * RAD_PER_DEG;
  _gRate[2] =               (gz / 131.0f - _gyroBias[2]) * RAD_PER_DEG;

  float alpha = FILTER_TAU / (FILTER_TAU + dt);

  // Median filter the accel angle reference to reject single-sample spikes
  // before they enter the complementary filter blend.  The gyro path is unaffected.
  _accelRollBuf[_accelBufIdx]  = atan2f(a_y, a_z);
  _accelPitchBuf[_accelBufIdx] = atan2f(-a_x, sqrtf(a_y*a_y + a_z*a_z));
  _accelBufIdx = (_accelBufIdx + 1) % 3;

  float accelRoll  = median3(_accelRollBuf[0],  _accelRollBuf[1],  _accelRollBuf[2])  - _rollOffset;
  float accelPitch = median3(_accelPitchBuf[0], _accelPitchBuf[1], _accelPitchBuf[2]) - _pitchOffset;

  _roll  = alpha * (_roll  + _gRate[0] * dt) + (1.0f - alpha) * accelRoll;
  _pitch = alpha * (_pitch + _gRate[1] * dt) + (1.0f - alpha) * accelPitch;

  // Update Z-axis bias while not actively yawing so thermal drift doesn't accumulate. 0.005 rad/s gate (~0.3 deg/s) ignores real rotation.
  if (fabsf(_gRate[2]) < 0.005f)
    _gyroBias[2] += (gz / 131.0f - _gyroBias[2]) * YAW_BIAS_LEARN;

  _yaw += _gRate[2] * dt;
}

float IMU::getRoll()  const { return _roll;        }
float IMU::getPitch() const { return _pitch;       }
float IMU::getYaw()   const { return _yaw;         }
float IMU::getGX()    const { return _gRate[0];    }
float IMU::getGY()    const { return _gRate[1];    }
float IMU::getGZ()    const { return _gRate[2];    }
float IMU::rawGX()    const { return _rawIMUG[0]; }
float IMU::rawGY()    const { return _rawIMUG[1]; }
float IMU::rawGZ()    const { return _rawIMUG[2]; }
float IMU::rawAX()    const { return _rawIMUA[0]; }
float IMU::rawAY()    const { return _rawIMUA[1]; }
float IMU::rawAZ()    const { return _rawIMUA[2]; }
