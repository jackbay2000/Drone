#include "IMU.h"

IMU::IMU(){
  for (int i = 0; i < 3; i++) {
    position[i] = 0.0f;
    velocity[i] = 0.0f;
  }
}

void IMU::setup(){
  //all necessary setup, save static acceleration
  Wire.begin();
  Wire.setClock(100000);
  imu.initialize();

  if(!imu.testConnection())
  {
    Serial.println("IMU init failed");
    while (1)
    {
      Serial.println("Check Setup");
      delay(1000);
    }
  }

  Serial.println("MPU Connection successful");

  lastTime = millis();

  //take 50 acceleration values to calculate static accel
  int16_t ax, ay, az;
  int16_t gx, gy, gz; //gyro values
  float init_accels[6] = {0};
  Serial.println("Logging static acceleration error, keep IMU still...");
  for (int i = 0; i < 50; i++)
  {
    imu.getAcceleration(&ax, &ay, &az);
    imu.getRotation(&gx, &gy, &gz);
    //convert to usable units and create cumulative sum
    init_accels[0] += ax / 16384.0;
    init_accels[1] += ay / 16384.0;
    init_accels[2] += az / 16384.0;
    init_accels[3] += gx / 131.0;
    init_accels[4] += gy / 131.0;
    init_accels[5] += gz / 131.0;
    delay(50); //~20 Hz update rate
  }

  static_accel[0] = init_accels[0] / 50.0;
  static_accel[1] = init_accels[1] / 50.0;
  static_accel[2] = init_accels[2] / 50.0;
  static_accel[3] = init_accels[3] / 50.0;
  static_accel[4] = init_accels[4] / 50.0;
  static_accel[5] = init_accels[5] / 50.0;

  Serial.print("Acceleration logging complete! Values (x, y, z, r, p, w) (gs): ");
  Serial.print(static_accel[0], 3); Serial.print(", ");
  Serial.print(static_accel[1], 3); Serial.print(", ");
  Serial.print(static_accel[2], 3); Serial.print(", ");
  Serial.print(static_accel[3], 3); Serial.print(", ");
  Serial.print(static_accel[4], 3); Serial.print(", ");
  Serial.print(static_accel[5], 3);

}


void IMU::update() {
  int16_t ax, ay, az;

  // --- Read sensor ---
  imu.getAcceleration(&ax, &ay, &az);

  // --- Convert to g's ---
  float accel[3];
  accel[0] = ax / 16384.0f;
  accel[1] = ay / 16384.0f;
  accel[2] = az / 16384.0f;

  // --- Subtract static bias (you already computed this) ---
  accel[0] -= static_accel[0];
  accel[1] -= static_accel[1];
  accel[2] -= static_accel[2];

  // --- Compute dt ---
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  // --- Integrate ---
  for (int i = 0; i < 3; i++) {
    velocity[i] += accel[i] * dt;
    position[i] += velocity[i] * dt;
  }
}

float IMU::getX() const {return position[0]; }
float IMU::getY() const {return position[1]; }
float IMU::getZ() const {return position[2]; }

void IMU::reset() {
  for (int i = 0; i < 3; i++) {
    position[i] = 0.0f;
    velocity[i] = 0.0f;
  }
}