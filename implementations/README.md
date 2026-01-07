# Implementations Directory

This directory contains **platform-specific implementations** of the HAL interfaces.

## Structure

```
implementations/
├── arduino/       ← Arduino/Teensy/ESP32 implementations
│   ├── Arduino*.h
│   └── Arduino*.cpp
└── stm32/         ← Template for future STM32 implementations
    └── (To be implemented)
```

## Arduino Implementations

All Arduino implementations:

- Inherit from abstract interfaces in `abstractions/`
- Use Adafruit and RadioHead libraries
- Are specific to Arduino-based boards (Teensy, ESP32, etc.)
- Can be conditionally excluded from builds

### Available Implementations

#### Sensors
- `ArduinoBmp280Sensor` - Barometric pressure/temperature (Adafruit BMP280)
- `ArduinoAccelGyroSensor` - Accelerometer/gyroscope (Adafruit LSM6DSOX)
- `ArduinoMagnetometerSensor` - Magnetometer (Adafruit LIS3MDL)
- `ArduinoTemperatureSensor` - Precision temperature (Adafruit MCP9808)
- `ArduinoGpsSensor` - GPS/GNSS (Adafruit GPS module)

#### Radios
- `ArduinoRFM95Radio` - 915MHz/433MHz LoRa (RadioHead RH_RF95)
- `ArduinoRYLR998Radio` - Serial LoRa module (AT command based)

## Using Arduino Implementations

### Basic Usage
```cpp
#include "implementations/arduino/ArduinoBmp280Sensor.h"

// Create instance
auto bmp280 = std::make_unique<ArduinoBmp280Sensor>(0x77, I2C_WIRE0);

// Initialize
bmp280->begin();

// Read data
auto* data = bmp280->read();
Serial.printf("Temp: %.2f°C\n", data->temperature);
```

### Using Abstract Interfaces
```cpp
#include "abstractions/IBmp280Sensor.h"
#include "implementations/arduino/ArduinoBmp280Sensor.h"

std::unique_ptr<IBmp280Sensor> sensor;
sensor = std::make_unique<ArduinoBmp280Sensor>(0x77);

// Interface usage - easy to switch implementations
sensor->begin();
auto* data = sensor->read();
```

## Constructor Signatures

### BMP280
```cpp
// I2C
ArduinoBmp280Sensor(uint8_t i2c_addr, uint8_t i2c_wire, float sea_level_hpa = 0.0f)

// SPI
ArduinoBmp280Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float sea_level_hpa = 0.0f)
```

### AccelGyro
```cpp
// I2C
ArduinoAccelGyroSensor(uint8_t i2c_addr, uint8_t i2c_wire)

// SPI
ArduinoAccelGyroSensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
```

### Magnetometer
```cpp
// I2C
ArduinoMagnetometerSensor(uint8_t i2c_addr, uint8_t i2c_wire)

// SPI
ArduinoMagnetometerSensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
```

### Temperature
```cpp
// I2C only
ArduinoTemperatureSensor(uint8_t i2c_addr, uint8_t i2c_wire)
```

### GPS
```cpp
ArduinoGpsSensor(uint8_t serial_port, uint32_t baud_rate = 9600)
```

### RFM95 Radio
```cpp
ArduinoRFM95Radio(uint8_t csPin, uint8_t intPin, uint8_t spiIndex = 1, float frequency = 915.0)
```

### RYLR998 Radio
```cpp
ArduinoRYLR998Radio(uint8_t serialPort, uint32_t baudRate = 9600)
```

## I2C Wire Definitions

```cpp
#define I2C_WIRE0   0  // Wire   (default SDA=18, SCL=19)
#define I2C_WIRE1   1  // Wire1  (SDA=17, SCL=16)
#define I2C_WIRE2   2  // Wire2  (SDA=25, SCL=24)
```

## Dependencies

These implementations require:
- Arduino framework (Teensy, ESP32, etc.)
- Adafruit_BMP280
- Adafruit_LSM6DS
- Adafruit_LIS3MDL
- Adafruit_MCP9808
- Adafruit_GPS
- Adafruit_BusIO
- RadioHead
- SPI library
- Wire (I2C) library

See `platformio.ini` for full dependency list.

## Adding New Arduino Implementations

1. Create header: `ArduinoNewSensor.h`
2. Inherit from appropriate interface: `INewSensor`
3. Implement all virtual methods:
   - `begin()`
   - `getStatus()`
   - `read()`
4. Create cpp file: `ArduinoNewSensor.cpp`
5. Add to CMakeLists.txt
6. Create mock in `mocks/` directory

## STM32 Template

The `stm32/` directory is a template for future STM32 implementations:

```cpp
// implementations/stm32/STM32Bmp280Sensor.h
#include "abstractions/IBmp280Sensor.h"

class STM32Bmp280Sensor : public IBmp280Sensor {
    // STM32-specific implementation
};
```

To add STM32 support:
1. Copy template structure
2. Replace Arduino calls with STM32 HAL calls
3. Update CMakeLists.txt