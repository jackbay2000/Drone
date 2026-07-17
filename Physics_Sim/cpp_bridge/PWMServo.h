#pragma once

// ---------------------------------------------------------------------------
// PWMServo stub — records writeMicroseconds() calls so the bridge can read
// ESC output values back out and convert them to normalized [0, 1].
// Shadows Teensy/Arduino's <PWMServo.h> (included by Drone_Main/controller.h).
// ---------------------------------------------------------------------------

#define MAX_SERVOS 8
extern int _servo_values[MAX_SERVOS];
extern int _servo_count;

class PWMServo {
public:
    PWMServo() : _index(-1) {}

    void attach(int /*pin*/, int /*min_us*/ = 544, int /*max_us*/ = 2400) {
        _index = _servo_count;
        if (_servo_count < MAX_SERVOS) {
            _servo_values[_servo_count] = 1000;
            ++_servo_count;
        }
    }

    void writeMicroseconds(int us) {
        if (_index >= 0 && _index < MAX_SERVOS)
            _servo_values[_index] = us;
    }

    int readMicroseconds() const {
        return (_index >= 0 && _index < MAX_SERVOS) ? _servo_values[_index] : 1000;
    }

private:
    int _index;
};
