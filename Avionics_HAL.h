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
#include "abstractions/WorkQueue.h"

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

#elif defined(USST_PLATFORM_STM32)
    // STM32 implementations
    #include "implementations/stm32/STM32Bmp280Sensor.h"
    #include "implementations/stm32/STM32AccelGyroSensor.h"
    #include "implementations/stm32/STM32MagnetometerSensor.h"
    #include "implementations/stm32/STM32TemperatureSensor.h"
    #include "implementations/stm32/STM32GpsSensor.h"
    #include "implementations/stm32/STM32RFM95Radio.h"
    #include "implementations/stm32/STM32RYLR998Radio.h"


#elif defined(USST_PLATFORM_MOCK)
    // Mock implementations for testing
    #include "implementations/mock/MockBmp280Sensor.h"
    #include "implementations/mock/MockAccelGyroSensor.h"
    #include "implementations/mock/MockMagnetometerSensor.h"
    #include "implementations/mock/MockTemperatureSensor.h"
    #include "implementations/mock/MockGpsSensor.h"

#else
    #error "No USST_PLATFORM defined! Set USST_PLATFORM in CMakeLists.txt to: ARDUINO, STM32, or MOCK"
#endif
