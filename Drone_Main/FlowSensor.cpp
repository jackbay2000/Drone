#include "FlowSensor.h"

FlowSensor::FlowSensor(uint8_t csPin) : _flow(csPin) {}

void FlowSensor::setup() {
  if (!_flow.begin()) {
    Serial.println("PMW3901 not found — check CS pin and wiring");
    while (1) delay(500);
  }
  Serial.println("PMW3901 ready");
}

void FlowSensor::read(int16_t &dx, int16_t &dy) {
  _flow.readMotionCount(&dx, &dy);
}
