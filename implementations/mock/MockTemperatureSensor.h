#pragma once

#include "abstractions/ITemperatureSensor.h"
#include "types.h"
#include <cstdint>

namespace HAL {

class TemperatureSensor : public ITemperatureSensor
{
private:
    TemperatureData data;
    SensorStatus status;

public:
    TemperatureSensor(uint8_t /*i2c_addr*/ = 0x18, uint8_t /*i2c_wire*/ = 0)
        : status(SensorStatus::Success), data{20.5f}
    {
    }

    virtual ~TemperatureSensor() = default;

    SensorStatus begin() override
    {
        return status;
    }

    SensorStatus getStatus() const override
    {
        return status;
    }

    const TemperatureData& read() override
    {
        return data;
    }

    void setData(float temperature)
    {
        data.temperature = temperature;
    }

    void setStatus(SensorStatus newStatus)
    {
        status = newStatus;
    }
};

} // namespace HAL
