# Mocks Directory

This directory contains **simple mock implementations** for testing without hardware.

## Overview

The mocks in this directory are meant for:

- Quick unit testing
- Desktop development
- Testing without hardware
- Deterministic test data

If you need advanced mocking features, see [HAL_DOCUMENTATION.md](../HAL_DOCUMENTATION.md#5-testing) for Google Mock integration.

## Available Mocks

- `MockBmp280Sensor.h` - Mock barometric sensor
- `MockAccelGyroSensor.h` - Mock accelerometer/gyroscope
- `MockMagnetometerSensor.h` - Mock magnetometer
- `MockTemperatureSensor.h` - Mock temperature sensor
- `MockGpsSensor.h` - Mock GPS sensor

## Basic Usage

### Simple Mock
```cpp
#include "mocks/MockBmp280Sensor.h"

auto sensor = std::make_unique<MockBmp280Sensor>();
sensor->begin();
sensor->setMockData(25.0f, 101325.0f, 0.0f);
auto* data = sensor->read();

ASSERT_FLOAT_EQ(data->temperature, 25.0f);
ASSERT_FLOAT_EQ(data->pressure, 101325.0f);
```

### With Standard Testing
```cpp
#include <cassert>
#include "mocks/MockAccelGyroSensor.h"

void test_accel_gyro() {
    auto sensor = std::make_unique<MockAccelGyroSensor>();
    
    AccelGyroData test_data{
        20.0f,   // temperature
        0.0f, 0.0f, 9.81f,  // acceleration (Z = gravity)
        0.0f, 0.0f, 0.0f    // angular velocity
    };
    
    ASSERT_EQ(sensor->begin(), 1);
    sensor->setMockData(test_data);
    
    auto* data = sensor->read();
    assert(data->accelZ == 9.81f);
}
```

## Mock Methods

### Common to All Sensors
```cpp
uint8_t begin()              // Always returns 1 (initialized)
uint8_t getStatus() const    // Always returns status value
<Data>* read()               // Returns pointer to mock data
```

### Mutable Methods
```cpp
void setMockData(...)        // Set return value for read()
void setStatus(uint8_t s)    // Change status return value
```

## Data Structures

Mocks use the same data structures as real implementations (from `types.h`).