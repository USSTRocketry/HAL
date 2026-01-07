#pragma once

// Core HAL headers (platform-independent)
#include "PINS.h"
#include "Storage.h"
#include "Chrono.h"
#include "types.h"

// Abstract interfaces (always included)
#include "abstractions/ISensor.h"
#include "abstractions/IBmp280Sensor.h"
#include "abstractions/IAccelGyroSensor.h"
#include "abstractions/IMagnetometerSensor.h"
#include "abstractions/ITemperatureSensor.h"
#include "abstractions/IGpsSensor.h"
#include "abstractions/ITelemetryRadio.h"

// Platform-specific implementations
#if defined(USST_PLATFORM_ARDUINO)
    // Arduino implementations
    #include "Arduino.h"
    #include "implementations/arduino/ArduinoBmp280Sensor.h"
    #include "implementations/arduino/ArduinoAccelGyroSensor.h"
    #include "implementations/arduino/ArduinoMagnetometerSensor.h"
    #include "implementations/arduino/ArduinoTemperatureSensor.h"
    #include "implementations/arduino/ArduinoGpsSensor.h"
    #include "implementations/arduino/ArduinoRFM95Radio.h"
    #include "implementations/arduino/ArduinoRYLR998Radio.h"
    #include "DebugLights.h"
    
    // Type aliases for easy use
    namespace HAL {
        using Bmp280Sensor = ArduinoBmp280Sensor;
        using AccelGyroSensor = ArduinoAccelGyroSensor;
        using MagnetometerSensor = ArduinoMagnetometerSensor;
        using TemperatureSensor = ArduinoTemperatureSensor;
        using GpsSensor = ArduinoGpsSensor;
        using RFM95Radio = ArduinoRFM95Radio;
        using RYLR998Radio = ArduinoRYLR998Radio;
    }

#elif defined(USST_PLATFORM_STM32)
    // STM32 implementations
    #include "implementations/stm32/STM32Bmp280Sensor.h"
    #include "implementations/stm32/STM32AccelGyroSensor.h"
    #include "implementations/stm32/STM32MagnetometerSensor.h"
    #include "implementations/stm32/STM32TemperatureSensor.h"
    #include "implementations/stm32/STM32GpsSensor.h"
    #include "implementations/stm32/STM32RFM95Radio.h"
    #include "implementations/stm32/STM32RYLR998Radio.h"
    
    // Type aliases for easy use
    namespace HAL {
        using Bmp280Sensor = STM32Bmp280Sensor;
        using AccelGyroSensor = STM32AccelGyroSensor;
        using MagnetometerSensor = STM32MagnetometerSensor;
        using TemperatureSensor = STM32TemperatureSensor;
        using GpsSensor = STM32GpsSensor;
        using RFM95Radio = STM32RFM95Radio;
        using RYLR998Radio = STM32RYLR998Radio;
    }

#elif defined(USST_PLATFORM_MOCK)
    // Mock implementations for testing
    #include "mocks/MockBmp280Sensor.h"
    #include "mocks/MockAccelGyroSensor.h"
    #include "mocks/MockMagnetometerSensor.h"
    #include "mocks/MockTemperatureSensor.h"
    #include "mocks/MockGpsSensor.h"
    
    // Type aliases for easy use
    namespace HAL {
        using Bmp280Sensor = MockBmp280Sensor;
        using AccelGyroSensor = MockAccelGyroSensor;
        using MagnetometerSensor = MockMagnetometerSensor;
        using TemperatureSensor = MockTemperatureSensor;
        using GpsSensor = MockGpsSensor;
        // No mock radios yet - to be added
    }

#elif defined(USST_PLATFORM_DESKTOP)
    // Desktop build - use mocks by default
    #include "mocks/MockBmp280Sensor.h"
    #include "mocks/MockAccelGyroSensor.h"
    #include "mocks/MockMagnetometerSensor.h"
    #include "mocks/MockTemperatureSensor.h"
    #include "mocks/MockGpsSensor.h"
    
    // Type aliases for easy use
    namespace HAL {
        using Bmp280Sensor = MockBmp280Sensor;
        using AccelGyroSensor = MockAccelGyroSensor;
        using MagnetometerSensor = MockMagnetometerSensor;
        using TemperatureSensor = MockTemperatureSensor;
        using GpsSensor = MockGpsSensor;
        // No mock radios yet - to be added
    }

#else
    #error "No USST_PLATFORM defined! Set USST_PLATFORM in CMakeLists.txt to: ARDUINO, STM32, MOCK, or DESKTOP"
#endif
