#include "implementations/arduino/ArduinoMagnetometerSensor.h"
#include <Adafruit_LIS3MDL.h>

// Macro definitions from Sensor.h
#define I2C_WIRE0   0 // default (SDA = 18, SCL = 19)
#define I2C_WIRE1   1 // (SDA = 17, SCL = 16)
#define I2C_WIRE2   2 // (SDA = 25, SCL = 24)

#define MAP_I2C_WIRE(w) (w == 0 ? Wire : (w == 1 ? Wire1 : Wire2))

#define SENSOR_MODE_I2C 0
#define SENSOR_MODE_SPI 1

// Static instance of Adafruit_LIS3MDL
static Adafruit_LIS3MDL lis3mdl;
static sensors_event_t mag_event;

ArduinoMagnetometerSensor::ArduinoMagnetometerSensor(uint8_t i2c_addr, uint8_t i2c_wire)
    : i2c_addr(i2c_addr), i2c_wire(i2c_wire), sensor_mode(SENSOR_MODE_I2C), status(0)
{
}

ArduinoMagnetometerSensor::ArduinoMagnetometerSensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
    : spi_cs(cs), spi_miso(miso), spi_mosi(mosi), spi_sck(sck), sensor_mode(SENSOR_MODE_SPI), status(0)
{
}

ArduinoMagnetometerSensor::~ArduinoMagnetometerSensor() {}

uint8_t ArduinoMagnetometerSensor::begin()
{
    if (sensor_mode == SENSOR_MODE_I2C) {
        status = lis3mdl.begin_I2C(i2c_addr, &MAP_I2C_WIRE(i2c_wire));
    } else {
        status = lis3mdl.begin_SPI(spi_cs, spi_sck, spi_miso, spi_mosi);
    }

    if (status) {
        // Configure LIS3MDL with default settings or custom settings.
        lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
        lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    }

    return status;
}

uint8_t ArduinoMagnetometerSensor::getStatus() const
{
    return status;
}

MagnetometerData* ArduinoMagnetometerSensor::read()
{
    lis3mdl.getEvent(&mag_event);

    data.magneticX = mag_event.magnetic.x;
    data.magneticY = mag_event.magnetic.y;
    data.magneticZ = mag_event.magnetic.z;

    return &data;
}
