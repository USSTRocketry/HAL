#pragma once
#include <cstdint>

namespace HAL {

/**
 * @brief Status enum for sensor initialization and operations
 */
enum class SensorStatus : uint8_t {
    Failure = 0,
    Success = 1
};

struct BMP280Data {
    float temperature;
    float pressure;
    float altitude;
};

struct AccelGyroData {
    float temperature;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};

struct MagnetometerData {
    float magneticX;
    float magneticY;
    float magneticZ;
};

struct TemperatureData {
    float temperature;
};

struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    float speed;       // in knots
    float angle;       // course over ground
    uint8_t satellites;
    uint8_t fix_quality;  // GPS signal quality
};

} // namespace HAL

