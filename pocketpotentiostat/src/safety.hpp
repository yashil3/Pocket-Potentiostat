#pragma once
#include <Arduino.h>

struct Safety {
  float vmin = 0.0f, vmax = 2.8f;  // clamp DAC
  float imax = 0.010f;             // 10 mA absolute
  bool tripped = false;

  inline float clampV(float v) const { return constrain(v, vmin, vmax); }

  inline void checkI(float iA) {
    if (fabsf(iA) > imax) tripped = true;
  }
};
