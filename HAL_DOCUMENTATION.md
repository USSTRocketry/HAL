# USST Avionics HAL — User Guide

---

## 1. Overview

- **What it is:** A Hardware Abstraction Layer exposing interfaces for sensors and radios.
- **Why it helps:** Same client code across Arduino, STM32 (future), and Mock builds.
- **How it’s structured:**
  - `abstractions/` — Pure interfaces (platform-agnostic)
  - `implementations/<platform>/` — Platform-specific classes (e.g., Arduino, Mock)
  - `Avionics_HAL.h` — Auto-selects the right implementation via platform macro

---

## 2. Quick Start

### 2.1 Include the HAL
```cpp
#include "Avionics_HAL.h"

// HAL types resolve to the correct implementation for your platform
HAL::Bmp280Sensor bmp(0x77, I2C_WIRE0);
HAL::AccelGyroSensor accel(0x6A, I2C_WIRE0);
HAL::GpsSensor gps(1, 9600);
HAL::RFM95Radio radio(10, 11, HW_SPI1);
```

### 2.2 Select Your Platform (USST_PLATFORM)

- The HAL selects implementations using compile definitions:
  - `USST_PLATFORM_ARDUINO`
  - `USST_PLATFORM_STM32` (template)
  - `USST_PLATFORM_MOCK`

PlatformIO `platformio.ini`:
```ini
build_flags =
  -DUSST_PLATFORM_ARDUINO
```

CMake (project-level):
```cmake
set(USST_PLATFORM "ARDUINO") # or STM32, MOCK
add_subdirectory(lib/HAL)
```

Command line:
```bash
cmake -DUSST_PLATFORM=ARDUINO ..
cmake -DUSST_PLATFORM=MOCK ..
```

---

## 3. Using the HAL

### 3.1 Sensors (HAL namespace)

BMP280 — Barometric Pressure/Temperature
```cpp
HAL::Bmp280Sensor bmp(i2c_addr, i2c_wire);
bmp.begin();
BMP280Data* d = bmp.read(); // temperature, pressure, altitude
```

LSM6DSOX — Accelerometer/Gyroscope
```cpp
HAL::AccelGyroSensor accel(i2c_addr, i2c_wire);
accel.begin();
AccelGyroData* d = accel.read(); // ax, ay, az, gx, gy, gz
```

LIS3MDL — Magnetometer
```cpp
HAL::MagnetometerSensor mag(i2c_addr, i2c_wire);
mag.begin();
MagnetometerData* d = mag.read(); // mx, my, mz
```

MCP9808 — Temperature
```cpp
HAL::TemperatureSensor temp(i2c_addr, i2c_wire);
temp.begin();
TemperatureData* d = temp.read(); // temperature
```

GPS/GNSS
```cpp
HAL::GpsSensor gps(serial_port, baud_rate);
gps.begin();
GPSData* d = gps.read(); // lat, lon, altitude, sats, fix, ...
```

I2C wires (Teensy typical):
```cpp
#define I2C_WIRE0 0
#define I2C_WIRE1 1
#define I2C_WIRE2 2
```

### 3.2 Radios (HAL namespace)

RFM95 — LoRa Radio
```cpp
HAL::RFM95Radio radio(cs, int_pin, spi_index, frequency);
radio.begin();
radio.setTxPower(20);
radio.send(data, length);
size_t rxLen radio.receive(buffer, maxLen, rxLen);
```

RYLR998 — Serial LoRa Module
```cpp
HAL::RYLR998Radio radio(serial_port, baud_rate);
radio.begin();
radio.setAddress(1);
radio.setDestinationAddress(2);
radio.send(data, length);
```

### 3.3 Complete Arduino Example
```cpp
#include "Avionics_HAL.h"

HAL::Bmp280Sensor bmp(0x77, I2C_WIRE0);
HAL::AccelGyroSensor accel(0x6A, I2C_WIRE0);
HAL::MagnetometerSensor mag(0x1E, I2C_WIRE0);
HAL::GpsSensor gps(1, 9600);
HAL::RFM95Radio radio(10, 11, HW_SPI1, 915.0);

void setup() {
  Serial.begin(115200);
  bmp.begin(); accel.begin(); mag.begin(); gps.begin(); radio.begin();
  radio.setTxPower(20); radio.setAddress(1);
}

void loop() {
  auto* b = bmp.read();
  auto* a = accel.read();
  auto* m = mag.read();
  auto* g = gps.read();
  uint8_t pkt[64] = {};
  radio.send(pkt, sizeof(pkt));
  delay(1000);
}
```

---

## 4. Build & Run

Arduino (PlatformIO):
```bash
pio run -e teensy41
```

Mock (CMake):
```bash
cmake -B build -DUSST_PLATFORM=MOCK
cmake --build build
```

---

## 5. Testing

### 5.1 Simple Mocks (No Google Mock)
```cpp
#include "Avionics_HAL.h" // with USST_PLATFORM=MOCK

HAL::Bmp280Sensor sensor;
sensor.begin();
sensor.setMockData(25.0f, 101325.0f, 100.0f);
BMP280Data* d = sensor.read();
```

### 5.2 Google Mock Integration

Basic mock class:
```cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "abstractions/IBmp280Sensor.h"

class MockBmp280 : public IBmp280Sensor {
public:
  MOCK_METHOD(uint8_t, begin, (), (override));
  MOCK_METHOD(uint8_t, getStatus, (), (const, override));
  MOCK_METHOD(BMP280Data*, read, (), (override));
};
```

Patterns:
```cpp
TEST(SensorTest, InitOk) {
  MockBmp280 s;
  EXPECT_CALL(s, begin()).WillOnce(testing::Return(1));
  EXPECT_TRUE(s.begin());
}

TEST(SensorTest, RepeatedReads) {
  MockBmp280 s;
  EXPECT_CALL(s, read())
    .WillOnce(testing::Invoke([](){ static BMP280Data d{20.5f,101325.0f,0.0f}; return &d; }))
    .WillOnce(testing::Invoke([](){ static BMP280Data d{21.0f,101326.0f,0.0f}; return &d; }));
  auto* d1 = s.read(); auto* d2 = s.read();
  EXPECT_FLOAT_EQ(d1->temperature, 20.5f);
  EXPECT_FLOAT_EQ(d2->temperature, 21.0f);
}
```

Running tests (CMake example):
```cmake
find_package(GTest REQUIRED)
find_package(gmock REQUIRED)
add_executable(hal_tests test_bmp280.cpp)
target_link_libraries(hal_tests HAL GTest::gtest GTest::gmock gmock_main)
```

Run:
```bash
ctest --test-dir build
```

---

## 6. Advanced

### 6.1 Using Interfaces Directly (Polymorphism)
```cpp
#include "Avionics_HAL.h"

std::unique_ptr<IBmp280Sensor> bmp;
std::unique_ptr<IAccelGyroSensor> accel;

bmp   = std::make_unique<HAL::Bmp280Sensor>(0x77, I2C_WIRE0);
accel = std::make_unique<HAL::AccelGyroSensor>(0x6A, I2C_WIRE0);
```

### 6.2 Compile-Time Platform Detection
```cpp
#if defined(USST_PLATFORM_ARDUINO)
  // Arduino-specific code (e.g., Serial)
#elif defined(USST_PLATFORM_STM32)
  // STM32 HAL specifics
#elif defined(USST_PLATFORM_MOCK)
  // Desktop/testing
#endif
```

---

## 7. Troubleshooting

- "No USST_PLATFORM defined!": Ensure the correct `build_flags` or CMake `-DUSST_PLATFORM=...` is set.
- Sensor init fails: Verify I2C address/wire, wiring, power, pull-ups.
- Radio issues: Check SPI pins, antenna, frequency compatibility with the ground station.
- Link errors (Arduino): Ensure dependencies are installed in `platformio.ini`.

---

## 8. Contributing

- To add sensors, radios, or platforms, follow the step-by-step guide in `CONTRIBUTING.md`.
- All data structures live in `types.h`.
- Interfaces are in `abstractions/` and must stay platform-agnostic.

---

## 9. References

- Google Mock: https://google.github.io/googletest/
- PlatformIO: https://docs.platformio.org/
- Teensy/Arduino libs: Adafruit sensors, RadioHead
