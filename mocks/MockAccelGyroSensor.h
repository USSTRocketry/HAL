#pragma once

#include "abstractions/IAccelGyroSensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Mock implementation of AccelGyro sensor for testing purposes.
 */
class MockAccelGyroSensor : public IAccelGyroSensor
{
private:
    AccelGyroData data;
    uint8_t status;

public:
    MockAccelGyroSensor()
        : status(1), data{20.0f, 0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f}
    {
    }

    virtual ~MockAccelGyroSensor() = default;

    uint8_t begin() override
    {
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    AccelGyroData* read() override
    {
        return &data;
    }

    void setMockData(const AccelGyroData& mockData)
    {
        data = mockData;
    }

    void setStatus(uint8_t newStatus)
    {
        status = newStatus;
    }
};
