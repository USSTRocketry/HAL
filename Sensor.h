#pragma once

#include <cstdint>

constexpr auto I2C_WIRE0 = 0; // default (SDA = 18, SCL = 19)
constexpr auto I2C_WIRE1 = 1; // (SDA = 17, SCL = 16)
constexpr auto I2C_WIRE2 = 2; // (SDA = 25, SCL = 24)

constexpr auto MAP_I2C_WIRE(uint8_t W)
{
    switch (W)
    {
        case 0:
            return Wire;
        case 1:
            return Wire1;
        default:
            return Wire2;
    }
}

constexpr auto SENSOR_MODE_I2C = 0;
constexpr auto SENSOR_MODE_SPI = 1;

class Sensor
{
protected:
    // I2C
    uint8_t i2c_addr;
    uint8_t i2c_wire;

    // SPI
    uint8_t spi_cs;
    uint8_t spi_miso;
    uint8_t spi_mosi;
    uint8_t spi_sck;

    uint8_t sensor_mode;
    uint8_t status;

public:
    Sensor(uint8_t i2c_addr, uint8_t i2c_wire);
    Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
    ~Sensor();
};
