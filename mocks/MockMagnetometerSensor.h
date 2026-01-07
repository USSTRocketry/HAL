#pragma once

#include "abstractions/IMagnetometerSensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Mock implementation of Magnetometer sensor for testing purposes.
 */
class MockMagnetometerSensor : public IMagnetometerSensor
{
private:
    MagnetometerData data;
    uint8_t status;

public:
    MockMagnetometerSensor()
        : status(1), data{0.0f, 0.0f, 0.0f}
    {
    }

    virtual ~MockMagnetometerSensor() = default;

    uint8_t begin() override
    {
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    MagnetometerData* read() override
    {
        return &data;
    }

    void setMockData(float magX, float magY, float magZ)
    {
        data.magneticX = magX;
        data.magneticY = magY;
        data.magneticZ = magZ;
    }

    void setStatus(uint8_t newStatus)
    {
        status = newStatus;
    }
};
