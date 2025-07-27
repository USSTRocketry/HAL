#include "Sensor.h"

Sensor::Sensor(uint8_t i2c_addr, uint8_t i2c_wire = I2C_WIRE0)
: i2c_addr(i2c_addr)
, i2c_wire(i2c_wire)
{
    sensor_mode = SensorMode::I2C;
}

Sensor::Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
: spi_cs(cs)
, spi_miso(miso)
, spi_mosi(mosi)
, spi_sck(sck)
{
    sensor_mode = SensorMode::SPI;
}

Sensor::~Sensor()
{
}