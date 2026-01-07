#include "implementations/arduino/ArduinoBmp280Sensor.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// Macro definitions from Sensor.h
#define I2C_WIRE0   0 // default (SDA = 18, SCL = 19)
#define I2C_WIRE1   1 // (SDA = 17, SCL = 16)
#define I2C_WIRE2   2 // (SDA = 25, SCL = 24)

#define MAP_I2C_WIRE(w) (w == 0 ? Wire : (w == 1 ? Wire1 : Wire2))

#define SENSOR_MODE_I2C 0
#define SENSOR_MODE_SPI 1

// Static instance of Adafruit_BMP280 (note: this is a limitation of the current design)
static Adafruit_BMP280 bmp;

ArduinoBmp280Sensor::ArduinoBmp280Sensor(uint8_t i2c_addr, uint8_t i2c_wire, float sea_level_hpa)
    : i2c_addr(i2c_addr), i2c_wire(i2c_wire), sensor_mode(SENSOR_MODE_I2C), 
      status(0), sea_level_hpa(sea_level_hpa)
{
    bmp = Adafruit_BMP280(&MAP_I2C_WIRE(i2c_wire));
}

ArduinoBmp280Sensor::ArduinoBmp280Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float sea_level_hpa)
    : spi_cs(cs), spi_miso(miso), spi_mosi(mosi), spi_sck(sck), sensor_mode(SENSOR_MODE_SPI),
      status(0), sea_level_hpa(sea_level_hpa)
{
    bmp = Adafruit_BMP280(cs, mosi, miso, sck);
}

ArduinoBmp280Sensor::~ArduinoBmp280Sensor() {}

uint8_t ArduinoBmp280Sensor::begin()
{
    if (sensor_mode == SENSOR_MODE_I2C) {
        status = bmp.begin(i2c_addr);
    } else {
        status = bmp.begin();
    }

    if (status) {
        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }

    return status;
}

uint8_t ArduinoBmp280Sensor::getStatus() const
{
    return status;
}

BMP280Data* ArduinoBmp280Sensor::read()
{
    if (sea_level_hpa > 0.1f) { // 0.1 because... float...
        data.altitude = bmp.readAltitude(sea_level_hpa);
    } else {
        data.altitude = bmp.readAltitude();
    }
    data.pressure = bmp.readPressure();
    data.temperature = bmp.readTemperature();
    return &data;
}
