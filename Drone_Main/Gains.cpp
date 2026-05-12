#include "Gains.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// How long to wait before the first line arrives.
// After each received line, a shorter 2 s inter-line timeout applies so the
// loader exits quickly once a paste is complete.
static constexpr unsigned long INITIAL_TIMEOUT_MS   = 8000UL;
static constexpr unsigned long INTERLINE_TIMEOUT_MS = 2000UL;

static void parseLine(const char* line, Gains& g) {
  const char* eq = strchr(line, '=');
  if (!eq) return;

  char key[32];
  int keyLen = (int)(eq - line);
  if (keyLen <= 0 || keyLen >= (int)sizeof(key)) return;
  memcpy(key, line, keyLen);
  key[keyLen] = '\0';
  for (int i = 0; key[i]; i++) key[i] = (char)toupper((unsigned char)key[i]);

  float val = atof(eq + 1);

  if      (!strcmp(key, "KP_ROLL"))  g.kp_roll  = val;
  else if (!strcmp(key, "KI_ROLL"))  g.ki_roll  = val;
  else if (!strcmp(key, "KD_ROLL"))  g.kd_roll  = val;
  else if (!strcmp(key, "KP_PITCH")) g.kp_pitch = val;
  else if (!strcmp(key, "KI_PITCH")) g.ki_pitch = val;
  else if (!strcmp(key, "KD_PITCH")) g.kd_pitch = val;
  else if (!strcmp(key, "KP_YAW"))   g.kp_yaw   = val;
  else if (!strcmp(key, "KI_YAW"))   g.ki_yaw   = val;
  else if (!strcmp(key, "KD_YAW"))   g.kd_yaw   = val;
  else if (!strcmp(key, "KP_X"))     g.kp_x     = val;
  else if (!strcmp(key, "KI_X"))     g.ki_x     = val;
  else if (!strcmp(key, "KD_X"))     g.kd_x     = val;
  else if (!strcmp(key, "KP_Y"))     g.kp_y     = val;
  else if (!strcmp(key, "KI_Y"))     g.ki_y     = val;
  else if (!strcmp(key, "KD_Y"))     g.kd_y     = val;
  else if (!strcmp(key, "KP_Z"))     g.kp_z     = val;
  else if (!strcmp(key, "KI_Z"))     g.ki_z     = val;
  else if (!strcmp(key, "KD_Z"))     g.kd_z     = val;
  else { Serial.print(F("Unknown key: ")); Serial.println(key); }
}

void printGains(const Gains& g) {
  Serial.println(F("--- Gains ---"));
  Serial.print(F("KP_ROLL="));  Serial.println(g.kp_roll,  4);
  Serial.print(F("KI_ROLL="));  Serial.println(g.ki_roll,  4);
  Serial.print(F("KD_ROLL="));  Serial.println(g.kd_roll,  4);
  Serial.print(F("KP_PITCH=")); Serial.println(g.kp_pitch, 4);
  Serial.print(F("KI_PITCH=")); Serial.println(g.ki_pitch, 4);
  Serial.print(F("KD_PITCH=")); Serial.println(g.kd_pitch, 4);
  Serial.print(F("KP_YAW="));   Serial.println(g.kp_yaw,   4);
  Serial.print(F("KI_YAW="));   Serial.println(g.ki_yaw,   4);
  Serial.print(F("KD_YAW="));   Serial.println(g.kd_yaw,   4);
  Serial.print(F("KP_X="));     Serial.println(g.kp_x,     4);
  Serial.print(F("KI_X="));     Serial.println(g.ki_x,     4);
  Serial.print(F("KD_X="));     Serial.println(g.kd_x,     4);
  Serial.print(F("KP_Y="));     Serial.println(g.kp_y,     4);
  Serial.print(F("KI_Y="));     Serial.println(g.ki_y,     4);
  Serial.print(F("KD_Y="));     Serial.println(g.kd_y,     4);
  Serial.print(F("KP_Z="));     Serial.println(g.kp_z,     4);
  Serial.print(F("KI_Z="));     Serial.println(g.ki_z,     4);
  Serial.print(F("KD_Z="));     Serial.println(g.kd_z,     4);
}

Gains loadGains() {
  Gains g;

  Serial.println(F("=== Gains loader ==="));
  Serial.println(F("Paste gains.txt into the serial monitor (8 s window)."));
  Serial.println(F("Lines starting with '#' are ignored. Waiting..."));

  char buf[64];
  int  bufLen         = 0;
  bool receivedAny    = false;
  unsigned long deadline = millis() + INITIAL_TIMEOUT_MS;

  while (millis() < deadline) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        buf[bufLen] = '\0';
        if (bufLen > 0 && buf[bufLen - 1] == '\r') buf[--bufLen] = '\0';
        if (bufLen > 0 && buf[0] != '#') parseLine(buf, g);
        bufLen = 0;
        // shorten the remaining window once data is flowing
        receivedAny = true;
        deadline = millis() + INTERLINE_TIMEOUT_MS;
      } else if (c != '\r' && bufLen < 63) {
        buf[bufLen++] = c;
      }
    }
  }

  if (!receivedAny) Serial.println(F("No gains received — using defaults."));
  printGains(g);
  return g;
}
