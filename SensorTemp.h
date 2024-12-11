#pragma once

#include <Wire.h>
#include <Adafruit_MCP9808.h>

#include "types.h"
#include "Sensor.h"

class SensorTemperature : Sensor
{
private:
    Adafruit_MCP9808 mcp9808;

public:
    float temperature;

public:
    SensorTemperature(uint8_t i2c_addr = MCP9808_I2CADDR_DEFAULT, uint8_t i2c_wire = I2C_WIRE0);
    ~SensorTemperature();
    uint8_t begin();
    float read();
};
