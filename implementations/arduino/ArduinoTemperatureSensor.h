#pragma once

#include "abstractions/ITemperatureSensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Arduino-specific implementation of Temperature sensor (MCP9808).
 * 
 * This implementation uses the Adafruit MCP9808 library and Arduino I2C
 * interface. For non-Arduino platforms, implement ITemperatureSensor directly.
 */
class ArduinoTemperatureSensor : public ITemperatureSensor
{
private:
    uint8_t i2c_addr;
    uint8_t i2c_wire;
    uint8_t status;
    TemperatureData data;

public:
    /**
     * @brief Construct a Temperature sensor for I2C communication.
     */
    ArduinoTemperatureSensor(uint8_t i2c_addr, uint8_t i2c_wire);

    virtual ~ArduinoTemperatureSensor();

    uint8_t begin() override;
    uint8_t getStatus() const override;
    TemperatureData* read() override;
};
