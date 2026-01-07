#pragma once

#include "abstractions/IMagnetometerSensor.h"
#include "types.h"
#include <cstdint>

class MockMagnetometerSensor : public IMagnetometerSensor
{
private:
    MagnetometerData data;
    uint8_t status;

public:
    MockMagnetometerSensor(uint8_t /*i2c_addr*/ = 0x1E, uint8_t /*i2c_wire*/ = 0)
        : status(1), data{0.0f, 0.0f, 0.0f}
    {
    }

    MockMagnetometerSensor(uint8_t /*cs*/, uint8_t /*miso*/, uint8_t /*mosi*/, uint8_t /*sck*/)
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
