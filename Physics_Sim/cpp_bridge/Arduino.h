#pragma once
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Basic Arduino types
// ---------------------------------------------------------------------------
typedef unsigned char byte;
typedef bool boolean;

// ---------------------------------------------------------------------------
// Simulated time — advanced from Python via sim_advance_time()
// ---------------------------------------------------------------------------
extern unsigned long _sim_micros;
inline unsigned long micros() { return _sim_micros; }
inline unsigned long millis() { return _sim_micros / 1000UL; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}

// ---------------------------------------------------------------------------
// Digital I/O stubs
// ---------------------------------------------------------------------------
#define HIGH 1
#define LOW  0
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int  digitalRead(uint8_t) { return 0; }

// ---------------------------------------------------------------------------
// Serial stub — templates absorb any argument type
// ---------------------------------------------------------------------------
struct _SerialStub {
    template<typename T> void print(T)        {}
    template<typename T> void print(T, int)   {}
    template<typename T> void println(T)      {}
    template<typename T> void println(T, int) {}
    void println() {}
    void begin(long) {}
    int  available() { return 0; }
    char read()      { return 0; }
};
extern _SerialStub Serial;

// ---------------------------------------------------------------------------
// F() string macro — PROGMEM wrapper, no-op on desktop
// ---------------------------------------------------------------------------
#define F(s) s

// ---------------------------------------------------------------------------
// Math helpers Arduino re-exports
// ---------------------------------------------------------------------------
#ifndef fabsf
#define fabsf(x) ((float)fabs((double)(x)))
#endif
#ifndef sqrtf
#define sqrtf(x) ((float)sqrt((double)(x)))
#endif
