#pragma once

#include "abstractions/ITemperatureSensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Mock implementation of Temperature sensor for testing purposes.
 */
class MockTemperatureSensor : public ITemperatureSensor
{
private:
    TemperatureData data;
    uint8_t status;

public:
    MockTemperatureSensor()
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
