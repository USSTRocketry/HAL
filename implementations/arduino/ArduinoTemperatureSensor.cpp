#include "implementations/arduino/ArduinoTemperatureSensor.h"
#include <Adafruit_MCP9808.h>

// Macro definitions from Sensor.h
#define I2C_WIRE0   0 // default (SDA = 18, SCL = 19)
#define I2C_WIRE1   1 // (SDA = 17, SCL = 16)
#define I2C_WIRE2   2 // (SDA = 25, SCL = 24)

#define MAP_I2C_WIRE(w) (w == 0 ? Wire : (w == 1 ? Wire1 : Wire2))

// Static instance of Adafruit_MCP9808
static Adafruit_MCP9808 mcp9808;

namespace HAL {

TemperatureSensor::TemperatureSensor(uint8_t i2c_addr, uint8_t i2c_wire)
    : i2c_addr(i2c_addr), i2c_wire(i2c_wire), status(SensorStatus::Failure)
{
}

TemperatureSensor::~TemperatureSensor() {}

SensorStatus TemperatureSensor::begin()
{
    // Initialize the MCP9808 sensor using the selected I2C wire and address
    bool initSuccess = mcp9808.begin(i2c_addr, &MAP_I2C_WIRE(i2c_wire));
    status = initSuccess ? SensorStatus::Success : SensorStatus::Failure;
    return status;
}

SensorStatus TemperatureSensor::getStatus() const
{
    return status;
}

const TemperatureData& TemperatureSensor::read()
{
    // Read the temperature in Celsius
    data.temperature = mcp9808.readTempC();
    return data;
}

} // namespace HAL
