#pragma once
#include <Arduino.h>

struct ChronoParams {
  float holdV = 0.0f;     // V vs RE
  uint32_t preMs = 1000;  // ms
  float stepV = 0.2f;     // V step for CA
  uint32_t stepMs = 5000; // ms at step
};

struct CVParams {
  float startV = -0.2f;
  float vertexV = 0.6f;
  float endV = -0.2f;
  float scanRate = 0.05f; // V/s
  float dt = 0.002f;      // s (500 Hz sample/update)
};

struct WaveformState {
  float targetV = 0;
  bool done = false;
};

class Chrono {
public:
  ChronoParams p;
  explicit Chrono(const ChronoParams& cp): p(cp) {}

  void reset() { t0 = millis(); stage = 0; }
  WaveformState step() {
    uint32_t t = millis() - t0;
    WaveformState s{};
    if (stage == 0 && t < p.preMs) {
      s.targetV = p.holdV;
    } else if (stage == 0) {
      stage = 1; t0 = millis();
    }
    if (stage == 1) {
      s.targetV = p.holdV + p.stepV;
      if (millis() - t0 > p.stepMs) { s.done = true; }
    }
    return s;
  }
private:
  uint32_t t0 = 0;
  int stage = 0;
};

class CV {
public:
  CVParams p;
  explicit CV(const CVParams& cp): p(cp) {}

  void reset() { t = 0; v = p.startV; dir = (p.vertexV>=p.startV)? +1 : -1; finished = false; last = millis(); }
  WaveformState step() {
    WaveformState s{};
    if (finished) { s.done = true; s.targetV = v; return s; }
    uint32_t now = millis();
    float dt_s = (now - last) / 1000.0f;
    if (dt_s < p.dt) { s.targetV = v; return s; }
    last = now;
    v += dir * p.scanRate * p.dt;
    if ((dir>0 && v >= p.vertexV) || (dir<0 && v <= p.vertexV)) { v = p.vertexV; dir = -dir; }
    if ((dir<0 && v <= p.endV)) { v = p.endV; finished = true; }
    s.targetV = v;
    return s;
  }
private:
  float v=0; int dir=+1; float t=0; bool finished=false; uint32_t last=0;
};
