#include "SensorMagnetometer.h"

SensorMagnetometer::SensorMagnetometer(uint8_t i2c_addr, uint8_t i2c_wire)
: Sensor(i2c_addr, i2c_wire)
{}

SensorMagnetometer::SensorMagnetometer(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
: Sensor(cs, miso, mosi, sck)
{}

SensorMagnetometer::~SensorMagnetometer() {}

uint8_t SensorMagnetometer::begin()
{
    if (sensor_mode == SENSOR_MODE_I2C)
    {
        status = lis3mdl.begin_I2C(i2c_addr, &I2C_WIRE(i2c_wire));
    }
    else
    {
        status = lis3mdl.begin_SPI(spi_cs, spi_sck, spi_miso, spi_mosi);
    }

    if (status)
    {
        // Configure LIS3MDL with default settings or custom settings.
        lis3mdl.setPerformanceMode(Adafruit_LIS3MDL::LIS3MDL_MEDIUMMODE);
        lis3mdl.setOperationMode(Adafruit_LIS3MDL::LIS3MDL_CONTINUOUSMODE);
        lis3mdl.setDataRate(Adafruit_LIS3MDL::LIS3MDL_DATARATE_155_HZ);
        lis3mdl.setRange(Adafruit_LIS3MDL::LIS3MDL_RANGE_4_GAUSS);
    }

    return status;
}

MagnetometerData* SensorMagnetometer::read()
{
    lis3mdl.getEvent(&mag_event);

    data.magneticX   = mag_event.magnetic.x;
    data.magneticY   = mag_event.magnetic.y;
    data.magneticZ   = mag_event.magnetic.z;

    return &data;
}
