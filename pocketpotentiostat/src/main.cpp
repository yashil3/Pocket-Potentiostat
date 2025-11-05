#include <Arduino.h>
#include "hw.hpp"

HW hw;

// Safety & calibration (editable)
struct Cal {
  float adc_offset_V = 0.0f;    // measured ADC offset (V)
  float adc_scale = 1.0f;       // scale factor if you measure a known voltage and it's off
  float dac_offset_V = 0.0f;    // DAC offset correction
} cal;

struct Safety {
  float vmin = 0.0f;
  float vmax = 3.0f;      // clamp DAC to safe range (slightly under 3.3)
  float imax = 0.020f;    // 20 mA safety trip default (tune per your analog front-end)
  bool tripped = false;
  void checkCurrent(float I) {
    if (fabs(I) > imax) tripped = true;
  }
} safety;

uint32_t t0;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.println("# Starting potentiostat firmware");

  hw.hasDac0 = true;
  hw.hasDac1 = false;   
  hw.adsGain = GAIN_ONE; 

  if (!hw.begin()) {
    Serial.println("# HW init failed - check I2C wiring and addresses");
    while (1) delay(1000);
  }

  cal.adc_offset_V = 0.0f;
  cal.adc_scale = 1.0f;
  cal.dac_offset_V = 0.0f;

  Serial.println("# Ready. Send commands: CA|CV|STOP|CAL");
  t0 = millis();
}

String inbuf;
enum LocalMode { IDLE, CA, CV };
LocalMode mode = IDLE;

void doCalibration(const String &line) {
  if (line.startsWith("CAL ADC")) {
    Serial.println("# CAL ADC starting: ensure WE is open-circuit (no current)");
    const int N=50;
    float acc = 0;
    for (int i=0;i<N;i++) {
      float v = hw.readAdcVoltsSingleEnded(0);
      acc += v;
      delay(10);
    }
    float mean = acc / N;
    cal.adc_offset_V = mean;
    Serial.print("# CAL ADC done offset(V)="); Serial.println(mean, 6);
  } else if (line.startsWith("CAL DAC")) {
    // Format: CAL DAC 1.234
    int sp = line.indexOf(' ', 7);
    String num = line.substring(8);
    float v = num.toFloat();
    cal.dac_offset_V = v;
    Serial.print("# CAL DAC offset set to "); Serial.println(v, 6);
  } else if (line.startsWith("CAL GAIN")) {
    // Format: CAL GAIN known_current_A measured_V
    Serial.println("# CAL GAIN not implemented in this simple script");
  } else {
    Serial.println("# CAL command not recognized");
  }
}

void parseCommandLine(const String &line) {
  if (line == "CA") {
    mode = CA;
    Serial.println("# Mode CA");
  } else if (line == "CV") {
    mode = CV;
    Serial.println("# Mode CV");
  } else if (line == "STOP") {
    mode = IDLE;
    hw.setDac0Volts(0.0f);
    Serial.println("# Mode STOP -> IDLE");
  } else if (line.startsWith("CAL")) {
    doCalibration(line);
  } else {
    Serial.println("# Unknown command");
  }
}


float cv_v = 0.0f;
int cv_dir = 1;
unsigned long cv_last_ms = 0;
const float cv_start = 0.0f, cv_vertex = 1.5f, cv_end = 0.0f;
const float cv_scan_rate_Vps = 0.05f; // 50 mV/s

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length()) {
        parseCommandLine(inbuf);
        inbuf = "";
      }
    } else inbuf += c;
  }

  if (safety.tripped) {
    hw.setDac0Volts(0.0f);
    Serial.println("# SAFETY TRIP - current exceeded. Send STOP to clear.");
    delay(200);
    return;
  }

  unsigned long now = millis();
  static unsigned long next_sample_ms = 0;
  const uint32_t sample_period_ms = 10; // 100 Hz sample rate
  if (now < next_sample_ms) return;
  next_sample_ms = now + sample_period_ms;

  float vset = 0.0f;
  if (mode == CA) {
    vset = 1.0f;
  } else if (mode == CV) {
    if (cv_last_ms == 0) { cv_last_ms = now; cv_v = cv_start; cv_dir = 1; }
    float dt = (now - cv_last_ms) / 1000.0f;
    if (dt >= 0.01f) { 
      cv_last_ms = now;
      cv_v += cv_dir * cv_scan_rate_Vps * dt;
      if (cv_dir>0 && cv_v >= cv_vertex) { cv_v = cv_vertex; cv_dir = -1; }
      if (cv_dir<0 && cv_v <= cv_end) { cv_v = cv_end; mode = IDLE; Serial.println("# CV DONE"); }
    }
    vset = cv_v;
  } else {
    vset = 0.0f;
  }

  
  vset = constrain(vset + cal.dac_offset_V, safety.vmin, safety.vmax);
  hw.setDac0Volts(vset);

  
  float vadc = hw.readAdcVoltsSingleEnded(0);
  float vadc_cal = (vadc - cal.adc_offset_V) * cal.adc_scale;
  float iA = hw.adcVoltsToCurrentA(vadc_cal);
  safety.checkCurrent(iA);

  // 
  Serial.print(now - t0); Serial.print(',');
  Serial.print(vset, 6); Serial.print(',');
  Serial.print(vadc, 6); Serial.print(',');
  Serial.print(vadc_cal, 6); Serial.print(',');
  Serial.println(iA, 12);
}
