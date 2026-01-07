#include "implementations/arduino/ArduinoTemperatureSensor.h"
#include <Adafruit_MCP9808.h>

// Macro definitions from Sensor.h
#define I2C_WIRE0   0 // default (SDA = 18, SCL = 19)
#define I2C_WIRE1   1 // (SDA = 17, SCL = 16)
#define I2C_WIRE2   2 // (SDA = 25, SCL = 24)

#define MAP_I2C_WIRE(w) (w == 0 ? Wire : (w == 1 ? Wire1 : Wire2))

// Static instance of Adafruit_MCP9808
static Adafruit_MCP9808 mcp9808;

ArduinoTemperatureSensor::ArduinoTemperatureSensor(uint8_t i2c_addr, uint8_t i2c_wire)
    : i2c_addr(i2c_addr), i2c_wire(i2c_wire), status(0)
{
}

ArduinoTemperatureSensor::~ArduinoTemperatureSensor() {}

uint8_t ArduinoTemperatureSensor::begin()
{
    // Initialize the MCP9808 sensor using the selected I2C wire and address
    status = mcp9808.begin(i2c_addr, &MAP_I2C_WIRE(i2c_wire));
    return status;
}

uint8_t ArduinoTemperatureSensor::getStatus() const
{
    return status;
}

TemperatureData* ArduinoTemperatureSensor::read()
{
    // Read the temperature in Celsius
    data.temperature = mcp9808.readTempC();
    return &data;
}
