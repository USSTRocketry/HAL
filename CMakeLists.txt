cmake_minimum_required(VERSION 3.28)
project(HAL VERSION 0.1 LANGUAGES CXX)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    SOVERSION 0
    VERSION 0.1
)

# Platform selection - set USST_PLATFORM to one of: ARDUINO, STM32, MOCK, DESKTOP
if(NOT DEFINED USST_PLATFORM)
    set(USST_PLATFORM "ARDUINO" CACHE STRING "Target platform (ARDUINO, STM32, MOCK, DESKTOP)")
endif()

message(STATUS "HAL: Building for platform: ${USST_PLATFORM}")

# Core abstraction and type definitions (always included)
set(HAL_CORE_SOURCES
    # Core header-only files are in abstractions/ directory
    # types.h is already included in target_include_directories
)

# Platform-specific implementations
if(USST_PLATFORM STREQUAL "ARDUINO")
    message(STATUS "HAL: Including Arduino implementations")
    list(APPEND HAL_CORE_SOURCES
        implementations/arduino/ArduinoBmp280Sensor.cpp
        implementations/arduino/ArduinoAccelGyroSensor.cpp
        implementations/arduino/ArduinoMagnetometerSensor.cpp
        implementations/arduino/ArduinoTemperatureSensor.cpp
        implementations/arduino/ArduinoGpsSensor.cpp
        implementations/arduino/ArduinoRFM95Radio.cpp
        implementations/arduino/ArduinoRYLR998Radio.cpp
        DebugLights.cpp
    )
elseif(USST_PLATFORM STREQUAL "STM32")
    message(STATUS "HAL: Including STM32 implementations")
    # Add STM32 implementation files here when ready
    list(APPEND HAL_CORE_SOURCES
        # implementations/stm32/STM32Bmp280Sensor.cpp
        # implementations/stm32/STM32AccelGyroSensor.cpp
        # ... etc
    )
elseif(USST_PLATFORM STREQUAL "MOCK")
    message(STATUS "HAL: Using Mock implementations (header-only)")
    # Mock implementations are header-only, no .cpp files needed
elseif(USST_PLATFORM STREQUAL "DESKTOP")
    message(STATUS "HAL: Desktop build - no hardware implementations")
    # Desktop builds use mocks or custom implementations
else()
    message(FATAL_ERROR "Unknown USST_PLATFORM: ${USST_PLATFORM}. Valid options: ARDUINO, STM32, MOCK, DESKTOP")
endif()

add_library(${PROJECT_NAME} STATIC
    ${HAL_CORE_SOURCES}
)

# Define the platform macro for compile-time selection
target_compile_definitions(${PROJECT_NAME} PUBLIC
    USST_PLATFORM_${USST_PLATFORM}
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/abstractions
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/implementations
)
