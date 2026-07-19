// Teensy 4.1 — ESC/prop test, all 4 motors together
// Signal -> pins 2,3,4,5 (M1 FL, M2 FR, M3 RL, M4 RR -- same mapping as
// Drone_Main/controller.cpp), ESC ground -> Teensy GND, ESC power -> LiPo
// (not Teensy 5V). Serial @ 115200: type 0-100 for throttle %, applied to
// all four motors simultaneously.

#include <PWMServo.h>

static constexpr uint8_t ESC_PINS[4] = {2, 3, 4, 5};
#define PWM_MIN  1000  // us, zero throttle
#define PWM_MAX  2000  // us, full throttle

PWMServo esc[4];

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    esc[i].attach(ESC_PINS[i], PWM_MIN, PWM_MAX);  // 0 deg -> PWM_MIN us, 180 deg -> PWM_MAX us
    esc[i].write(0);   // arm at zero throttle
  }
  delay(3000);
}

void loop() {
  if (!Serial.available()) return;

  int pct = constrain(Serial.parseInt(), 0, 100);
  int angle = map(pct, 0, 100, 0, 180);
  for (int i = 0; i < 4; i++) esc[i].write(angle);

  Serial.println(pct);
}
