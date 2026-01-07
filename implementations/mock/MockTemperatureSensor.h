#pragma once

#include "abstractions/ITemperatureSensor.h"
#include "types.h"
#include <cstdint>

class MockTemperatureSensor : public ITemperatureSensor
{
private:
    TemperatureData data;
    uint8_t status;

public:
    MockTemperatureSensor(uint8_t /*i2c_addr*/ = 0x18, uint8_t /*i2c_wire*/ = 0)
        : status(1), data{20.5f}
    {
    }

    virtual ~MockTemperatureSensor() = default;

    uint8_t begin() override
    {
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    TemperatureData* read() override
    {
        return &data;
    }

    void setMockData(float temperature)
    {
        data.temperature = temperature;
    }

    void setStatus(uint8_t newStatus)
    {
        status = newStatus;
    }
};
