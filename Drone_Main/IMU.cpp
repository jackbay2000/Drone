#include "IMU.h"
#include <math.h>

static constexpr float FILTER_TAU  = 2.0f;
static constexpr float ACCEL_GATE  = 0.15f;
static constexpr float RAD_PER_DEG = 0.01745329251f;

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

  const int N = 500;
  int16_t ax, ay, az, gx, gy, gz;
  double sa[3] = {}, sg[3] = {};

  Serial.println("Calibrating — keep still for ~2.5 s...");
  for (int i = 0; i < N; i++) {
    _imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sa[0] += ax / 16384.0;  sa[1] += ay / 16384.0;  sa[2] += az / 16384.0;
    sg[0] += gx / 131.0;    sg[1] += gy / 131.0;    sg[2] += gz / 131.0;
    delay(5);
  }

  float ax_avg = sa[0] / N, ay_avg = sa[1] / N, az_avg = sa[2] / N;
  _gyroBias[0] = sg[0] / N;
  _gyroBias[1] = sg[1] / N;
  _gyroBias[2] = sg[2] / N;

  _roll  = atan2f(ay_avg, az_avg);
  _pitch = atan2f(-ax_avg, sqrtf(ay_avg * ay_avg + az_avg * az_avg));
  _yaw   = 0.0f;

  _accelBias[0] = ax_avg - (-sinf(_pitch));
  _accelBias[1] = ay_avg - (cosf(_pitch) * sinf(_roll));
  _accelBias[2] = az_avg - (cosf(_pitch) * cosf(_roll));

  Serial.print("Calibration done.  Roll: ");  Serial.print(degrees(_roll),  1);
  Serial.print(" deg   Pitch: "); Serial.print(degrees(_pitch), 1); Serial.println(" deg");

  _lastMicros = micros();
}

void IMU::update() {
  int16_t ax, ay, az, gx, gy, gz;
  _imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  float dt = (now - _lastMicros) * 1.0e-6f;
  _lastMicros = now;
  if (dt <= 0.0f || dt > 0.1f) return;

  float a[3] = {
    ax / 16384.0f - _accelBias[0],
    ay / 16384.0f - _accelBias[1],
    az / 16384.0f - _accelBias[2]
  };

  _gRate[0] = (gx / 131.0f - _gyroBias[0]) * RAD_PER_DEG;
  _gRate[1] = (gy / 131.0f - _gyroBias[1]) * RAD_PER_DEG;
  _gRate[2] = (gz / 131.0f - _gyroBias[2]) * RAD_PER_DEG;

  float alpha = FILTER_TAU / (FILTER_TAU + dt);

  float cp = cosf(_pitch), sp = sinf(_pitch);
  float cr = cosf(_roll),  sr = sinf(_roll);

  float grav[3] = { -sp, cp * sr, cp * cr };
  float lin[3]  = { a[0] - grav[0], a[1] - grav[1], a[2] - grav[2] };
  float lin_mag = sqrtf(lin[0]*lin[0] + lin[1]*lin[1] + lin[2]*lin[2]);

  float roll_g  = _roll  + _gRate[0] * dt;
  float pitch_g = _pitch + _gRate[1] * dt;
  _yaw         += _gRate[2] * dt;

  if (lin_mag < ACCEL_GATE) {
    _roll  = alpha * roll_g  + (1.0f - alpha) * atan2f(a[1], a[2]);
    _pitch = alpha * pitch_g + (1.0f - alpha) * atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
  } else {
    _roll  = roll_g;
    _pitch = pitch_g;
  }
}

float IMU::getRoll()  const { return _roll;     }
float IMU::getPitch() const { return _pitch;    }
float IMU::getYaw()   const { return _yaw;      }
float IMU::getGX()    const { return _gRate[0]; }
float IMU::getGY()    const { return _gRate[1]; }
float IMU::getGZ()    const { return _gRate[2]; }
