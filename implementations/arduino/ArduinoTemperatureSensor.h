#pragma once

#include "abstractions/ITemperatureSensor.h"
#include "types.h"
#include <cstdint>

namespace HAL {

/**
 * @brief Arduino-specific implementation of Temperature sensor (MCP9808).
 * 
 * This implementation uses the Adafruit MCP9808 library and Arduino I2C
 * interface. For non-Arduino platforms, implement ITemperatureSensor directly.
 */
class TemperatureSensor : public ITemperatureSensor
{
private:
    uint8_t i2c_addr;
    uint8_t i2c_wire;
    SensorStatus status;
    TemperatureData data;

public:
    /**
     * @brief Construct a Temperature sensor for I2C communication.
     */
    TemperatureSensor(uint8_t i2c_addr, uint8_t i2c_wire);

    virtual ~TemperatureSensor();

    SensorStatus begin() override;
    SensorStatus getStatus() const override;
    const TemperatureData& read() override;
};

} // namespace HAL
