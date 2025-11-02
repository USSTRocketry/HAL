cmake_minimum_required(VERSION 3.28)
project(HAL VERSION 0.1 LANGUAGES CXX)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    SOVERSION 0
    VERSION 0.1
)

add_library(${PROJECT_NAME} STATIC
DebugLights.cpp
Gps.cpp
RFM95Radio.cpp
RYLR998Radio.cpp
Sensor.cpp
SensorAccelGyro.cpp
SensorBMP280.cpp
SensorMagnetometer.cpp
SensorTemp.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
