#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1015.h>



struct HW {

  const int SDA_PIN = 33;
  const int SCL_PIN = 35;

 
  Adafruit_MCP4725 dac0;
  Adafruit_MCP4725 dac1;
  bool hasDac0 = true;
  bool hasDac1 = false; 
  // ADS1115 ADC
  Adafruit_ADS1115 ads;
  adsGain_t adsGain = GAIN_ONE;

  
  uint8_t dac0Addr = 0x60; // default A0=GND
  uint8_t dac1Addr = 0x61; 
  uint8_t adsAddr  = 0x48; // default ADS1115 addr

  // scaling 
  float vref = 3.3f;      
  float tia_r = 10000.0f; 
  float adc_frontend_gain = 1.0f;

  bool begin() {
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(10);

    // init DAC 0
    if (hasDac0) {
      if (!dac0.begin(dac0Addr)) {
        Serial.println("# Error: DAC0 init failed");
        return false;
      }
    }
    // init DAC 1 if present
    if (hasDac1) {
      if (!dac1.begin(dac1Addr)) {
        Serial.println("# Error: DAC1 init failed");
        return false;
      }
    }

    // init ADS1115
    ads.begin();
    ads.setGain(adsGain);

    return true;
  }

  // Set DAC output voltage
  // code = round( (volts/vref) * 4095 )
  void setDac0Volts(float volts) {
    if (!hasDac0) return;
    volts = constrain(volts, 0.0f, vref);
    uint16_t code = (uint16_t) roundf((volts / vref) * 4095.0f);
    dac0.setVoltage(code, false);
  }

  void setDac1Volts(float volts) {
    if (!hasDac1) return;
    volts = constrain(volts, 0.0f, vref);
    uint16_t code = (uint16_t) roundf((volts / vref) * 4095.0f);
    dac1.setVoltage(code, false);
  }

  
  float readAdcVoltsSingleEnded(uint8_t channel) {
    int16_t raw = ads.readADC_SingleEnded(channel); // returns in counts
    float lsb;
    switch (adsGain) {
      case GAIN_TWOTHIRDS: lsb = 6.144f / 32768.0f; break;
      case GAIN_ONE:       lsb = 4.096f / 32768.0f; break;
      case GAIN_TWO:       lsb = 2.048f / 32768.0f; break;
      case GAIN_FOUR:      lsb = 1.024f / 32768.0f; break;
      case GAIN_EIGHT:     lsb = 0.512f / 32768.0f; break;
      case GAIN_SIXTEEN:   lsb = 0.256f / 32768.0f; break;
      default:             lsb = 4.096f / 32768.0f; break;
    }
    float volts = raw * lsb;
    return volts;
  }

  float adcVoltsToCurrentA(float vadc) {
    float v_tia = vadc / adc_frontend_gain;
    return v_tia / tia_r;
  }

  float readCurrentFromChannel0() {
    float vadc = readAdcVoltsSingleEnded(0);
    return adcVoltsToCurrentA(vadc);
  }
};
