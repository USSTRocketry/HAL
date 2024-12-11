#pragma once

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#include "types.h"
#include "Sensor.h"


class SensorBMP280: Sensor
{
private:
    Adafruit_BMP280 bmp;
    float   ground_alt;

public:
    BMP280Data data;

public:
    SensorBMP280(uint8_t i2c_addr = BMP280_ADDRESS, uint8_t i2c_wire = I2C_WIRE0, float ground_alt = 0.0f);
    SensorBMP280(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float ground_alt = 0.0f);
    uint8_t begin();
    BMP280Data* read();
    ~SensorBMP280();
};