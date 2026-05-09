// Teensy 4.1 — BLHeli_S ESC Test
// Library: PWMServo (PJRC, built into Teensyduino)
// PWMServo.write(val): val < 200 → degrees, val >= 200 → microseconds directly
// ESC PWM range: 1000us (min) to 2000us (max), 3.3V signal OK on BLHeli_S
//
// Wiring:
//   ESC signal wire --> Teensy pin 9
//   ESC ground      --> Teensy GND
//   ESC power       --> LiPo (do NOT power from Teensy 5V)
//
// Serial commands (115200 baud):
//   0-100  : set throttle to that percentage
//   ramp   : ramp from 0% to 30% and back (safe bench test)
//   stop   : immediately cut throttle to 0%

#include <PWMServo.h>

#define ESC_PIN      9
#define THROTTLE_MIN 1000  // microseconds — ESC minimum (arm / zero throttle)
#define THROTTLE_MAX 2000  // microseconds — ESC maximum

PWMServo esc;

// ── helpers ──────────────────────────────────────────────────────────────────

void setThrottleUS(int us) {
  us = constrain(us, THROTTLE_MIN, THROTTLE_MAX);
  esc.write(us);  // PWMServo: values >= 200 are treated as microseconds
}

void setThrottlePct(int pct) {
  pct = constrain(pct, 0, 100);
  int us = map(pct, 0, 100, THROTTLE_MIN, THROTTLE_MAX);
  setThrottleUS(us);
  Serial.print("Throttle: ");
  Serial.print(pct);
  Serial.print("%  (");
  Serial.print(us);
  Serial.println(" us)");
}

void armESC() {
  Serial.println("Arming ESC — holding minimum throttle for 3 s ...");
  setThrottleUS(THROTTLE_MIN);
  delay(3000);
  Serial.println("ESC armed. Ready.");
  Serial.println();
  Serial.println("Commands: 0-100 | ramp | stop");
}

void rampTest() {
  Serial.println("Ramp test: 0% -> 30% -> 0%  (safe bench value)");
  for (int i = 0; i <= 30; i++) {
    setThrottlePct(i);
    delay(80);
  }
  delay(2000);
  for (int i = 30; i >= 0; i--) {
    setThrottlePct(i);
    delay(80);
  }
  Serial.println("Ramp complete.");
}

// ── setup / loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("Teensy 4.1 — BLHeli_S ESC Test");
  Serial.println("================================");

  esc.attach(ESC_PIN);
  armESC();
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.equalsIgnoreCase("ramp")) {
    rampTest();
  } else if (input.equalsIgnoreCase("stop")) {
    setThrottlePct(0);
    Serial.println("Stopped.");
  } else {
    int pct = input.toInt();
    if (pct >= 0 && pct <= 100) {
      setThrottlePct(pct);
    } else {
      Serial.println("Unknown command. Use 0-100, ramp, or stop.");
    }
  }
}
