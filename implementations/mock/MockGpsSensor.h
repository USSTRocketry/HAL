#pragma once

#include "abstractions/IGpsSensor.h"
#include "types.h"
#include <cstdint>

class MockGpsSensor : public IGpsSensor
{
private:
    GPSData data;
    uint8_t status;

public:
    MockGpsSensor(uint8_t /*serial_port*/ = 1, uint32_t /*baud_rate*/ = 9600)
        : status(1), data{37.7749f, -122.4194f, 0.0f, 0.0f, 0.0f, 0, 0}
    {
    }

    virtual ~MockGpsSensor() = default;

    uint8_t begin() override
    {
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    GPSData* read() override
    {
        return &data;
    }

    void setMockData(const GPSData& mockData)
    {
        data = mockData;
    }

    void setStatus(uint8_t newStatus)
    {
        status = newStatus;
    }
};
