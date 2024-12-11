#pragma once

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>

#include "types.h"
#include "Sensor.h"

class SensorMagnetometer : Sensor
{
private:
    Adafruit_LIS3MDL lis3mdl;

    sensors_event_t mag_event;

public:
    MagnetometerData data;

public:
    SensorMagnetometer(uint8_t i2c_addr = LIS3MDL_I2CADDR_DEFAULT, uint8_t i2c_wire = I2C_WIRE0);
    SensorMagnetometer(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
    uint8_t begin();
    MagnetometerData* read();
    ~SensorMagnetometer();
};
