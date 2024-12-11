#include "SensorTemp.h"

SensorTemperature::SensorTemperature(uint8_t i2c_addr, uint8_t i2c_wire)
: Sensor(i2c_addr, i2c_wire)
{}

SensorTemperature::~SensorTemperature() {}

uint8_t SensorTemperature::begin()
{
    // Initialize the MCP9808 sensor using the selected I2C wire and address
    status = mcp9808.begin(i2c_addr, &I2C_WIRE(i2c_wire));
    return status;
}

float SensorTemperature::read()
{
    // Read the temperature in Celsius and store it
    temperature = mcp9808.readTempC();
    return temperature;
}
