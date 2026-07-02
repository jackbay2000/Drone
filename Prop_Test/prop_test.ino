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

  esc.attach(ESC_PIN);
  esc.write(PWM_MIN);   // arm
  delay(3000);
}

void loop() {
  if (!Serial.available()) return;

  //int pct = constrain(Serial.parseInt(), 0, 100);
  //int us = map(pct, 0, 100, PWM_MIN, PWM_MAX);
  esc.write(1000);
}
