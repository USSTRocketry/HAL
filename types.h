#pragma once
#include <cstdint>

// Define macros for selecting hardware serial ports and corresponding pins
constexpr auto HW_SERIAL1 = 1; // Serial1 (RX = 0, TX = 1)
constexpr auto HW_SERIAL2 = 2; // Serial2 (RX = 7, TX = 8)
constexpr auto HW_SERIAL3 = 3; // Serial3 (RX = 15, TX = 14)
constexpr auto HW_SERIAL4 = 4; // Serial4 (RX = 16, TX = 17)
constexpr auto HW_SERIAL5 = 5; // Serial5 (RX = 21, TX = 20)
constexpr auto HW_SERIAL6 = 6; // Serial6 (RX = 25, TX = 24)
constexpr auto HW_SERIAL7 = 7; // Serial7 (RX = 28, TX = 29)
constexpr auto HW_SERIAL8 = 8; // Serial8 (RX = 34, TX = 35)

constexpr inline auto HW_SERIAL(uint32_t Serial)
{
    switch (Serial)
    {
        case 1:
            return Serial1;
        case 2:
            return Serial2;
        case 3:
            return Serial3;
        case 4:
            return Serial4;
        case 5:
            return Serial5;
        case 6:
            return Serial6;
        case 7:
            return Serial7;
        default:
            return Serial8;
    }
}

constexpr auto HW_SPI0 = 0; // hardware_spi  (MISO = , MOSI = , SCK = )
constexpr auto HW_SPI1 = 1; // hardware_spi1 (MISO = , MOSI = , SCK = )
constexpr auto HW_SPI2 = 2; // hardware_spi2 (MISO = , MOSI = , SCK = )

struct BMP280Data
{
    float temperature = 0.f;
    float pressure    = 0.f;
    float altitude    = 0.f;
};

struct AccelGyroData
{
    float temperature = 0.f;
    float accelX      = 0.f;
    float accelY      = 0.f;
    float accelZ      = 0.f;
    float gyroX       = 0.f;
    float gyroY       = 0.f;
    float gyroZ       = 0.f;
};

struct MagnetometerData
{
    float magneticX = 0.f;
    float magneticY = 0.f;
    float magneticZ = 0.f;
};

struct TemperatureData
{
    float temperature = 0.f;
};

struct GPSData
{
    float latitude      = 0.f;
    float longitude     = 0.f;
    float altitude      = 0.f;
    float speed         = 0.f; // in knots
    float angle         = 0.f; // course over ground
    uint8_t satellites  = 0.f;
    uint8_t fix_quality = 0.f; // GPS signal quality
};
