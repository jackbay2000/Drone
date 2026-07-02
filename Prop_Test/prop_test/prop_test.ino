// Teensy 4.1 — ESC/prop test
// Signal -> pin 9, ESC ground -> Teensy GND, ESC power -> LiPo (not Teensy 5V)
// Serial @ 115200: type 0-100 for throttle %

#include <PWMServo.h>

#define ESC_PIN  9
#define PWM_MIN  1000  // us, zero throttle
#define PWM_MAX  2000  // us, full throttle

PWMServo esc;

void setup() {
  Serial.begin(115200);

  esc.attach(ESC_PIN, PWM_MIN, PWM_MAX);  // 0 deg -> PWM_MIN us, 180 deg -> PWM_MAX us
  esc.write(0);   // arm at zero throttle
  delay(3000);
}

void loop() {
  if (!Serial.available()) return;

  int pct = constrain(Serial.parseInt(), 0, 100);
  int angle = map(pct, 0, 100, 0, 180);
  esc.write(angle);

  Serial.println(pct);
}
