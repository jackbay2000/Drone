#include <PWMServo.h>

PWMServo Prop1;
PWMServo Prop2;
PWMServo Prop3;
PWMServo Prop4;

void setup() {
  // put your setup code here, to run once:
  Prop1.attach(9);
  Prop2.attach(10);
  Prop3.attach(11);
  Prop4.attach(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  Prop1.writeMicroseconds

}
