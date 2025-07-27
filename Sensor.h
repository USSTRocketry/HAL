#pragma once

#include <cstdint>

namespace I2C
{
enum Wire
{
    Wire0 = 0, // default (SDA = 18, SCL = 19)
    Wire1 = 1, // (SDA = 17, SCL = 16)
    Wire2 = 2, // (SDA = 25, SCL = 24)
};
} // namespace I2C

constexpr auto MAP_I2C_WIRE(uint8_t Wire)
{
    switch (Wire)
    {
        case 0:
            return Wire;
        case 1:
            return Wire1;
        default:
            return Wire2;
    }
}

enum class SensorMode : uint8_t
{
    I2C,
    SPI
};

class Sensor
{
public:
    Sensor(uint8_t i2c_addr, uint8_t i2c_wire);
    Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
    virtual ~Sensor() = default;

protected:
    // consider std::variant
    union
    {
        // I2C
        struct
        {
            uint8_t i2c_addr;
            uint8_t i2c_wire;
        };

        // SPI
        struct
        {
            uint8_t spi_cs;
            uint8_t spi_miso;
            uint8_t spi_mosi;
            uint8_t spi_sck;
        };
    };
    SensorMode sensor_mode;
    bool status;
};
