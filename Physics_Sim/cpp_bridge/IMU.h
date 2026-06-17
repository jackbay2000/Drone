#pragma once

// ---------------------------------------------------------------------------
// IMU stub — values are injected from Python via sim_set_state() each step.
// Exposes the same public API that AttitudeController and YawController use.
// ---------------------------------------------------------------------------

class IMU {
public:
    IMU() : _roll(0), _pitch(0), _yaw(0),
            _gx(0), _gy(0), _gz(0),
            _ax(0), _ay(0), _az(0) {}

    // Called by Drone_Main.ino — no-ops in simulation
    void setup() {}
    void update() {}

    // Setters called from sim_bridge.cpp
    void setRoll (float v) { _roll  = v; }
    void setPitch(float v) { _pitch = v; }
    void setYaw  (float v) { _yaw   = v; }
    void setGyro (float gx, float gy, float gz) { _gx=gx; _gy=gy; _gz=gz; }
    void setAccel(float ax, float ay, float az) { _ax=ax; _ay=ay; _az=az; }

    // Getters used by AttitudeController / YawController
    float getRoll()  const { return _roll;  }
    float getPitch() const { return _pitch; }
    float getYaw()   const { return _yaw;   }
    float rawGX()    const { return _gx;    }
    float rawGY()    const { return _gy;    }
    float rawGZ()    const { return _gz;    }
    float rawAX()    const { return _ax;    }
    float rawAY()    const { return _ay;    }
    float rawAZ()    const { return _az;    }

private:
    float _roll, _pitch, _yaw;
    float _gx, _gy, _gz;
    float _ax, _ay, _az;
};
