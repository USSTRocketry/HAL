#pragma once

#include "abstractions/IMagnetometerSensor.h"
#include "types.h"
#include <cstdint>

namespace HAL {

class MagnetometerSensor : public IMagnetometerSensor
{
private:
    MagnetometerData data;
    SensorStatus status;

public:
    MagnetometerSensor(uint8_t /*i2c_addr*/ = 0x1E, uint8_t /*i2c_wire*/ = 0)
        : status(SensorStatus::Success), data{0.0f, 0.0f, 0.0f}
    {
    }

    MagnetometerSensor(uint8_t /*cs*/, uint8_t /*miso*/, uint8_t /*mosi*/, uint8_t /*sck*/)
        : status(SensorStatus::Success), data{0.0f, 0.0f, 0.0f}
    {
    }

    virtual ~MagnetometerSensor() = default;

    SensorStatus begin() override
    {
        return status;
    }

    SensorStatus getStatus() const override
    {
        return status;
    }

    const MagnetometerData& read() override
    {
        return data;
    }

    void setData(float magX, float magY, float magZ)
    {
        data.magneticX = magX;
        data.magneticY = magY;
        data.magneticZ = magZ;
    }

    void setStatus(SensorStatus newStatus)
    {
        status = newStatus;
    }
};

} // namespace HAL
