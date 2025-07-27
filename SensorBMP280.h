#pragma once

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "types.h"
#include "Sensor.h"

class SensorBMP280 : Sensor
{
private:
    Adafruit_BMP280 bmp;
    float   sea_level_hpa;

public:
    BMP280Data data;

public:
    uint8_t begin();
    BMP280Data& read();
    ~SensorBMP280();

public:
    SensorBMP280(uint8_t i2c_addr = BMP280_ADDRESS, uint8_t i2c_wire = I2C_WIRE0, float sea_level_hpa = 0.0f);
    SensorBMP280(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float sea_level_hpa = 0.0f);
    ~SensorBMP280() override = default;
};
