# Potentiostat Firmware


## Hardware Components
- **ESP32 Dev Module** 
- **Adafruit MCP4725 DAC** (I2C address 0x60) for voltage output
- **Adafruit ADS1115 ADC** (I2C address 0x48) for current measurement
- **TIA (Trans-impedance Amplifier)** with 10kΩ resistor
- **REF3033 or 3.3V reference**

## File Structure
```
src/
├── main.cpp          # Main firmware with setup, loop, and command parsing
├── hw.hpp           # Hardware abstraction (DAC, ADC, calibration)
├── protocols.hpp    # Command parsing definitions
├── protocols.cpp    # Command parsing implementation
├── safety.hpp       # Safety limits and current monitoring
└── waveforms.hpp    # Waveform generation classes (CA, CV)
```

## Features

### Serial Commands
- `CA` - Chronoamperometry mode (constant potential)
- `CV` - Cyclic voltammetry mode  
- `STOP` - Stop measurement and set output to 0V
- `CAL ADC` - Calibrate ADC offset (requires open circuit)
- `CAL DAC <voltage>` - Set DAC offset correction

### Safety Features
- Current limiting (default 20mA, configurable)
- Voltage clamping (0V to 2.8V)
- Automatic shutdown on overcurrent

### Measurement Output
CSV format: `time_ms,voltage_set,voltage_measured,voltage_corrected,current_A`

## Configuration
Edit the parameters in `main.cpp` and `hw.hpp` to match your hardware:
- I2C pin assignments (SDA_PIN, SCL_PIN)
- I2C device addresses  
- TIA resistor value
- Reference voltage
- Safety limits
