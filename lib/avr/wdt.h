/*
* This file is only meant to stub Arduino functions for compatibility.
*/
#pragma once
#define INPUT_PULLUP 0x2
#define FALLING 0
#define INPUT 0x0
#define OUTPUT 0x1
#define HIGH 0x1
#define LOW 0x0

inline void pinMode(int a, int b) {}
inline void attachInterrupt(int a, void (*b)(void), int c) {}
inline float analogRead(double a) { return 512.0f; }
inline void digitalWrite(int a, int b) {}
inline void analogWrite(int a, int b) {}