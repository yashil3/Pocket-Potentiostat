#pragma once
#include <Arduino.h>

enum Mode { MODE_IDLE, MODE_CA, MODE_CV };

struct Command {
  Mode mode = MODE_IDLE;
};

bool parseCommand(const String& line, Command& cmd);
void printSample(uint32_t t_ms, float vset, float vadc, float iA);
