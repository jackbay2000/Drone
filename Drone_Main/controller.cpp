// #include "controller.h"

// PID::PID(float kp, float ki, float kd, float dt)
//   : Kp(kp), Ki(ki), Kd(kd), dt(dt), integral(0.0f) {}


// void PID::compute(const float* history, float des, float* out){
//   float pos = history[99];
//   float err = des - pos;

//   integral += err * dt;

//   float prev = pos[98];
//   float derivative = (pos - prev) / dt;

//   *out = (Kp * err) + (Ki * integral) - (Kd * derivative);

// }

// void PID::reset() {
//   integral = 0.0f
// }