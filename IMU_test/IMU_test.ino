#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

// ---------- Position + Velocity ----------
float posX = 0, posY = 0, posZ = 0;
float velX = 0, velY = 0, velZ = 0;

unsigned long lastTime = 0;

// ---------- Update Position ----------
void updatePosition(float ax, float ay, float az)
{
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // seconds
  lastTime = currentTime;

  if (dt <= 0) return;

  // Convert g → m/s^2
  float ax_ms = ax * 9.81;
  float ay_ms = ay * 9.81;
  float az_ms = az * 9.81;

  // Remove gravity (assumes Z axis is vertical)
  az_ms -= 9.81;

  // Deadband filter (reduce noise drift)
  if (abs(ax_ms) < 0.2) ax_ms = 0;
  if (abs(ay_ms) < 0.2) ay_ms = 0;
  if (abs(az_ms) < 0.2) az_ms = 0;

  // Integrate acceleration → velocity
  velX += ax_ms * dt;
  velY += ay_ms * dt;
  velZ += az_ms * dt;

  // Velocity damping (helps control drift)
  velX *= 0.98;
  velY *= 0.98;
  velZ *= 0.98;

  // Integrate velocity → position
  posX += velX * dt;
  posY += velY * dt;
  posZ += velZ * dt;
}

// ---------- Setup ----------
void setup()
{
  Serial.begin(115200);

  // Wait for serial (important for Teensy)
  while (!Serial) {}

  Serial.println("BOOTING MPU6050...");

  Wire.begin();
  Wire.setClock(100000); // safer I2C speed

  imu.initialize();

  if (!imu.testConnection())
  {
    Serial.println("IMU init failed!");
    while (1)
    {
      Serial.println("Check wiring / power");
      delay(1000);
    }
  }

  Serial.println("MPU6050 connected!");

  lastTime = millis(); // initialize timer
}

// ---------- Main Loop ----------
void loop()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);

  // Convert to usable units
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;

  // Update position tracking
  updatePosition(ax_g, ay_g, az_g);

  // ---------- Print Data ----------
  Serial.print("Acc (g): ");
  Serial.print(ax_g, 3); Serial.print(", ");
  Serial.print(ay_g, 3); Serial.print(", ");
  Serial.print(az_g, 3);

  Serial.print(" | Gyr (dps): ");
  Serial.print(gx_dps, 2); Serial.print(", ");
  Serial.print(gy_dps, 2); Serial.print(", ");
  Serial.print(gz_dps, 2);

  Serial.print(" | Pos (m): ");
  Serial.print(posX, 3); Serial.print(", ");
  Serial.print(posY, 3); Serial.print(", ");
  Serial.println(posZ, 3);

  delay(50); // ~20 Hz update rate
}

// #include <Wire.h>
// #include <MPU6050.h>
// #include <Arduino.h>

// MPU6050 imu;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) {}

//   Wire.begin();
//   Wire.setClock(100000); // start slower for stability

//   Serial.println("Initializing MPU6050...");

//   imu.initialize();

//   if (!imu.testConnection()) {
//     Serial.println("MPU6050 connection failed");
//     while (1) {
//       delay(1000);
//     }
//   }

//   Serial.println("MPU6050 connected!");
// }

// void loop() {
//   int16_t ax, ay, az;
//   int16_t gx, gy, gz;

//   imu.getAcceleration(&ax, &ay, &az);
//   imu.getRotation(&gx, &gy, &gz);

//   Serial.print("Acc: ");
//   Serial.print(ax); Serial.print(", ");
//   Serial.print(ay); Serial.print(", ");
//   Serial.print(az);

//   Serial.print(" | Gyr: ");
//   Serial.print(gx); Serial.print(", ");
//   Serial.print(gy); Serial.print(", ");
//   Serial.println(gz);

//   delay(200);
// }