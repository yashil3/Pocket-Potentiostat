#include "protocols.hpp"

bool parseCommand(const String& line, Command& cmd) {
  if (line.startsWith("CA")) { cmd.mode = MODE_CA; return true; }
  if (line.startsWith("CV")) { cmd.mode = MODE_CV; return true; }
  if (line.startsWith("STOP")) { cmd.mode = MODE_IDLE; return true; }
  return false;
}

void printSample(uint32_t t_ms, float vset, float vadc, float iA) {
  Serial.print(t_ms); Serial.print(',');
  Serial.print(vset, 5); Serial.print(',');
  Serial.print(vadc, 5); Serial.print(',');
  Serial.println(iA, 9);
}
