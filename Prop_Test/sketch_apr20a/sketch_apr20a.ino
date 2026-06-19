// Teensy 4.1 — 4-Motor ESC bench test
// Pins match Drone_Main motor assignment so no re-wiring is needed for flight:
//   M1 FL CCW = pin 2,  M2 FR CW  = pin 3
//   M3 RL CW  = pin 4,  M4 RR CCW = pin 5
//
// Serial commands (115200 baud):
//   0-100 : set all motors to that throttle %
//   ramp  : ramp all motors 0% -> 30% -> 0%  (safe bench value)
//   stop  : cut all motors immediately

#include <PWMServo.h>

#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000

PWMServo Prop1, Prop2, Prop3, Prop4;

void setAllUS(int us) {
  us = constrain(us, THROTTLE_MIN, THROTTLE_MAX);
  Prop1.write(us);
  Prop2.write(us);
  Prop3.write(us);
  Prop4.write(us);
}

void setAllPct(int pct) {
  pct = constrain(pct, 0, 100);
  int us = map(pct, 0, 100, THROTTLE_MIN, THROTTLE_MAX);
  setAllUS(us);
  Serial.print("Throttle: ");
  Serial.print(pct);
  Serial.print("%  (");
  Serial.print(us);
  Serial.println(" us)");
}

void armESCs() {
  Serial.println("Arming ESCs — holding minimum throttle for 3 s ...");
  setAllUS(THROTTLE_MIN);
  delay(3000);
  Serial.println("Armed. Ready.");
  Serial.println("Commands: 0-100 | ramp | stop");
}

void rampTest() {
  Serial.println("Ramp: 0% -> 30% -> 0%");
  for (int i = 0; i <= 30; i++) { setAllPct(i); delay(80); }
  delay(2000);
  for (int i = 30; i >= 0; i--) { setAllPct(i); delay(80); }
  Serial.println("Ramp complete.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("Teensy 4.1 — 4-Motor ESC Test");
  Serial.println("==============================");

  Prop1.attach(2);   // M1 FL CCW
  Prop2.attach(3);   // M2 FR CW
  Prop3.attach(4);   // M3 RL CW
  Prop4.attach(5);   // M4 RR CCW

  armESCs();
}

void loop() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.equalsIgnoreCase("ramp")) {
    rampTest();
  } else if (input.equalsIgnoreCase("stop")) {
    setAllPct(0);
    Serial.println("Stopped.");
  } else {
    int pct = input.toInt();
    if (pct >= 0 && pct <= 100) {
      setAllPct(pct);
    } else {
      Serial.println("Unknown command. Use 0-100, ramp, or stop.");
    }
  }
}
