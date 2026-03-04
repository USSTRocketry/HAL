# CMake support for Arduino implementation sources
# this directory is only added when USST_PLATFORM==ARDUINO.
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${PROJECT_NAME} PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoAccelGyroSensor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoBmp280Sensor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoGpsSensor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoMagnetometerSensor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoRFM95Radio.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoRYLR998Radio.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoTemperatureSensor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoWorkQueue.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/ArduinoDebugLights.cpp"
)