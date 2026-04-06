#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050.h>

class IMU {
  private:

    MPU6050 imu;

    unsigned long lastTime = 0;

    float position[3];
    float velocity[3];
    float static_accel[6];

  public:
    IMU();

    void setup();
    
    void update();

    float getX() const;
    float getY() const;
    float getZ() const;

    void reset();

};

#endif