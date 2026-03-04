#include "ArduinoBmp280Sensor.h"
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

namespace HAL {

// Static instance of Adafruit_BMP280 (note: this is a limitation of the current design)
static Adafruit_BMP280 bmp;

Bmp280Sensor::Bmp280Sensor(uint8_t i2c_addr, uint8_t i2c_wire, float sea_level_hpa)
    : i2c_addr(i2c_addr), i2c_wire(i2c_wire), sensor_mode(SENSOR_MODE_I2C), 
      status(SensorStatus::Failure), sea_level_hpa(sea_level_hpa)
{
    bmp = Adafruit_BMP280(&MAP_I2C_WIRE(i2c_wire));
}

Bmp280Sensor::Bmp280Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float sea_level_hpa)
    : spi_cs(cs), spi_miso(miso), spi_mosi(mosi), spi_sck(sck), sensor_mode(SENSOR_MODE_SPI),
      status(SensorStatus::Failure), sea_level_hpa(sea_level_hpa)
{
    bmp = Adafruit_BMP280(cs, mosi, miso, sck);
}

Bmp280Sensor::~Bmp280Sensor() {}

SensorStatus Bmp280Sensor::begin()
{
    bool initSuccess = false;
    if (sensor_mode == SENSOR_MODE_I2C) {
        initSuccess = bmp.begin(i2c_addr);
    } else {
        initSuccess = bmp.begin();
    }

    if (initSuccess) {
        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        status = SensorStatus::Success;
    } else {
        status = SensorStatus::Failure;
    }

    return status;
}

SensorStatus Bmp280Sensor::getStatus() const
{
    return status;
}

const BMP280Data& Bmp280Sensor::read()
{
    if (sea_level_hpa > 0.1f) { // 0.1 because... float...
        data.altitude = bmp.readAltitude(sea_level_hpa);
    } else {
        data.altitude = bmp.readAltitude();
    }
    data.pressure = bmp.readPressure();
    data.temperature = bmp.readTemperature();
    return data;
}

} // namespace HAL
