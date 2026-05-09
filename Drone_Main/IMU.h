

#include <Wire.h>
#include <MPU6050.h>

class IMU {
  private:

    MPU6050 imu;

    unsigned long lastTime = 0;

    float position[3];
    float velocity[3];
    float static_accel[6];  // [0-2]: electronic accel bias, [3-5]: gyro bias

    float roll  = 0.0f;     // radians, updated each call to update()
    float pitch = 0.0f;     // radians, updated each call to update()

    int zupt_count = 0;     // consecutive samples below ZUPT threshold

  public:
    IMU();

    void setup();
    
    void update();

    float getX() const;
    float getY() const;
    float getZ() const;

    void reset();

};
