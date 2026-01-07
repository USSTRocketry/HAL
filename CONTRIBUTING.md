# Contributing to HAL

This guide explains how to add new sensors, radios, or platform implementations to the HAL.

## Architecture Overview

The HAL uses a layered architecture:

```
User Code
    ↓
Avionics_HAL.h (automatic platform selection)
    ↓
Abstract Interfaces (abstractions/*.h)
    ↓
Platform Implementations (implementations/arduino/, implementations/stm32/, etc.)
    ↓
Hardware Libraries (Adafruit, RadioHead, STM32 HAL, etc.)
```

## Adding a New Sensor

### Step 1: Create the Abstract Interface

Create a new file in `abstractions/I<SensorName>Sensor.h`:

```cpp
#pragma once

#include "ISensor.h"
#include "types.h"

class IMyNewSensor : public ISensor {
public:
    virtual ~IMyNewSensor() = default;
    
    // Standard ISensor methods (inherited)
    virtual uint8_t begin() = 0;
    virtual uint8_t getStatus() const = 0;
    virtual MyNewSensorData* read() = 0;
    
    // Sensor-specific methods (if needed)
    virtual void calibrate() = 0;
    virtual void setSensitivity(uint8_t level) = 0;
};
```

### Step 2: Define the Data Structure

Add your data structure to `types.h`:

```cpp
struct MyNewSensorData {
    float value1;
    float value2;
    uint32_t timestamp;
};
```

### Step 3: Create the Arduino Implementation

Create header: `implementations/arduino/ArduinoMyNewSensor.h`

```cpp
#pragma once

#include "abstractions/IMyNewSensor.h"
#include <SomeAdafruitLibrary.h>

class ArduinoMyNewSensor : public IMyNewSensor {
private:
    SomeAdafruitClass sensor;
    MyNewSensorData data;
    uint8_t i2cAddr;
    uint8_t i2cWire;

public:
    // Constructor
    ArduinoMyNewSensor(uint8_t i2c_addr, uint8_t i2c_wire);
    
    // ISensor interface
    uint8_t begin() override;
    uint8_t getStatus() const override;
    MyNewSensorData* read() override;
    
    // Sensor-specific methods
    void calibrate() override;
    void setSensitivity(uint8_t level) override;
};
```

Create implementation: `implementations/arduino/ArduinoMyNewSensor.cpp`

```cpp
#include "ArduinoMyNewSensor.h"
#include <Wire.h>

ArduinoMyNewSensor::ArduinoMyNewSensor(uint8_t i2c_addr, uint8_t i2c_wire)
    : i2cAddr(i2c_addr), i2cWire(i2c_wire) {
}

uint8_t ArduinoMyNewSensor::begin() {
    // Select the I2C bus
    TwoWire* wire = (i2cWire == I2C_WIRE0) ? &Wire :
                    (i2cWire == I2C_WIRE1) ? &Wire1 :
                    (i2cWire == I2C_WIRE2) ? &Wire2 : &Wire;
    
    // Initialize the sensor
    if (!sensor.begin(i2cAddr, wire)) {
        return 0;  // Failure
    }
    
    return 1;  // Success
}

uint8_t ArduinoMyNewSensor::getStatus() const {
    return sensor.isConnected() ? 1 : 0;
}

MyNewSensorData* ArduinoMyNewSensor::read() {
    sensor.readData();
    
    data.value1 = sensor.getValue1();
    data.value2 = sensor.getValue2();
    data.timestamp = millis();
    
    return &data;
}

void ArduinoMyNewSensor::calibrate() {
    sensor.performCalibration();
}

void ArduinoMyNewSensor::setSensitivity(uint8_t level) {
    sensor.setSensitivity(level);
}
```

### Step 4: Create a Mock Implementation

Create `implementations/mock/MockMyNewSensor.h`:

```cpp
#pragma once

#include "abstractions/IMyNewSensor.h"

class MockMyNewSensor : public IMyNewSensor {
private:
    MyNewSensorData data;
    bool initialized = false;

public:
    uint8_t begin() override {
        initialized = true;
        return 1;
    }
    
    uint8_t getStatus() const override {
        return initialized ? 1 : 0;
    }
    
    MyNewSensorData* read() override {
        return &data;
    }
    
    void calibrate() override {
        // Mock - do nothing
    }
    
    void setSensitivity(uint8_t level) override {
        // Mock - do nothing
    }
    
    // Helper for testing
    void setMockData(float val1, float val2) {
        data.value1 = val1;
        data.value2 = val2;
        data.timestamp = 0;
    }
};
```

### Step 5: Update CMakeLists.txt

Add the implementation to the build:

```cmake
# Platform-specific implementations
if(USST_PLATFORM STREQUAL "ARDUINO")
    message(STATUS "HAL: Including Arduino implementations")
    list(APPEND HAL_CORE_SOURCES
        # ... existing files ...
        implementations/arduino/ArduinoMyNewSensor.cpp
    )
```

### Step 6: Update Avionics_HAL.h

Add the sensor to the platform selection:

```cpp
// Abstract interfaces (always included)
#include "abstractions/IMyNewSensor.h"

// Platform-specific implementations
#if defined(USST_PLATFORM_ARDUINO)
    #include "implementations/arduino/ArduinoMyNewSensor.h"
    
    namespace HAL {
        // ... existing aliases ...
        using MyNewSensor = ArduinoMyNewSensor;
    }

#elif defined(USST_PLATFORM_STM32)
    #include "implementations/stm32/STM32MyNewSensor.h"
    
    namespace HAL {
        using MyNewSensor = STM32MyNewSensor;
    }

#elif defined(USST_PLATFORM_MOCK)
    #include "implementations/mock/MockMyNewSensor.h"
    
    namespace HAL {
        using MyNewSensor = MockMyNewSensor;
    }
#endif
```

## Adding a New Platform (e.g., STM32)

### Step 1: Create Platform Directory

```bash
mkdir implementations/stm32
```

### Step 2: Implement Each Sensor for the Platform

For each existing sensor interface, create an STM32 implementation:

`implementations/stm32/STM32Bmp280Sensor.h`:
```cpp
#pragma once

#include "abstractions/IBmp280Sensor.h"
#include "stm32f4xx_hal.h"  // Or your STM32 HAL

class STM32Bmp280Sensor : public IBmp280Sensor {
private:
    I2C_HandleTypeDef* i2cHandle;
    BMP280Data data;
    uint8_t i2cAddr;

public:
    STM32Bmp280Sensor(I2C_HandleTypeDef* handle, uint8_t addr);
    
    uint8_t begin() override;
    uint8_t getStatus() const override;
    BMP280Data* read() override;
};
```

`implementations/stm32/STM32Bmp280Sensor.cpp`:
```cpp
#include "STM32Bmp280Sensor.h"

STM32Bmp280Sensor::STM32Bmp280Sensor(I2C_HandleTypeDef* handle, uint8_t addr)
    : i2cHandle(handle), i2cAddr(addr) {
}

uint8_t STM32Bmp280Sensor::begin() {
    // Use STM32 HAL to initialize sensor
    uint8_t initCmd[] = {0xF4, 0x27};  // Example: set ctrl_meas
    
    if (HAL_I2C_Master_Transmit(i2cHandle, i2cAddr << 1, initCmd, 2, 100) != HAL_OK) {
        return 0;
    }
    
    return 1;
}

uint8_t STM32Bmp280Sensor::getStatus() const {
    return 1;  // Implement actual status check
}

BMP280Data* STM32Bmp280Sensor::read() {
    uint8_t buffer[6];
    uint8_t reg = 0xF7;  // pressure/temp data register
    
    HAL_I2C_Master_Transmit(i2cHandle, i2cAddr << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(i2cHandle, i2cAddr << 1, buffer, 6, 100);
    
    // Parse and convert data
    // ... calculate temperature, pressure, altitude ...
    
    return &data;
}
```

### Step 3: Update CMakeLists.txt

```cmake
elseif(USST_PLATFORM STREQUAL "STM32")
    message(STATUS "HAL: Including STM32 implementations")
    list(APPEND HAL_CORE_SOURCES
        implementations/stm32/STM32Bmp280Sensor.cpp
        implementations/stm32/STM32AccelGyroSensor.cpp
        implementations/stm32/STM32MagnetometerSensor.cpp
        implementations/stm32/STM32TemperatureSensor.cpp
        implementations/stm32/STM32GpsSensor.cpp
        # ... etc
    )
```

### Step 4: Update Avionics_HAL.h

The STM32 section should already exist from the previous example - just verify it includes your new implementations.

## Adding a New Radio

Same process as sensors, but inherit from `ITelemetryRadio`:

```cpp
// abstractions/INewRadio.h (optional, or just use ITelemetryRadio)
#include "abstractions/ITelemetryRadio.h"

class ArduinoNewRadio : public ITelemetryRadio {
    // Implement all ITelemetryRadio methods
};
```

## Testing Your Changes

### 1. Test with Arduino Platform

```bash
cmake -DUSST_PLATFORM=ARDUINO -B build
cmake --build build
```

### 2. Test with Mock Platform

```bash
cmake -DUSST_PLATFORM=MOCK -B build
cmake --build build
```

### 3. Write Unit Tests

See [HAL_DOCUMENTATION.md](HAL_DOCUMENTATION.md#5-testing) for details on writing tests.

## Checklist

Before submitting your changes:

- [ ] Abstract interface created in `abstractions/`
- [ ] Data structure added to `types.h`
- [ ] Arduino implementation created in `implementations/arduino/`
- [ ] Mock implementation created in `implementations/mock/`
- [ ] CMakeLists.txt updated
- [ ] Avionics_HAL.h updated with platform selection
- [ ] Code compiles for ARDUINO platform
- [ ] Code compiles for MOCK platform
- [ ] Documentation updated (if needed)
- [ ] Example code written (in documentation)

## Style Guide

### Naming Conventions

- **Interfaces**: `I<Name>Sensor` or `I<Name>Radio` (e.g., `IBmp280Sensor`)
- **Arduino Implementations**: `Arduino<Name>Sensor` (e.g., `ArduinoBmp280Sensor`)
- **STM32 Implementations**: `STM32<Name>Sensor` (e.g., `STM32Bmp280Sensor`)
- **Mocks**: `Mock<Name>Sensor` (e.g., `MockBmp280Sensor`)
- **Data Structures**: `<Name>Data` (e.g., `BMP280Data`)

### File Organization

```
abstractions/
    ISensor.h               # Base interface
    IBmp280Sensor.h         # Specific sensor interface
    ITelemetryRadio.h       # Radio interface

implementations/
    arduino/
        ArduinoBmp280Sensor.h
        ArduinoBmp280Sensor.cpp
    stm32/
        STM32Bmp280Sensor.h
        STM32Bmp280Sensor.cpp

implementations/mock/
    MockBmp280Sensor.h      # Header-only mocks
```

### Code Style

- Use C++20 features
- Pure virtual interfaces in `abstractions/`
- No Arduino dependencies in `abstractions/`
- No platform-specific includes in `abstractions/`
- All platform-specific code in `implementations/<platform>/`
- Clear comments explaining hardware-specific behavior

## Questions?

- Check [HAL_DOCUMENTATION.md](HAL_DOCUMENTATION.md#2-quick-start) for usage examples
- See [HAL_DOCUMENTATION.md](HAL_DOCUMENTATION.md#5-testing) for testing guidance
- Look at existing implementations for reference
