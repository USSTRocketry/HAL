#pragma once

#include "abstractions/IGpsSensor.h"
#include "types.h"
#include <cstdint>

namespace HAL {

class GpsSensor : public IGpsSensor
{
private:
    GPSData data;
    SensorStatus status;

public:
    GpsSensor(uint8_t /*serial_port*/ = 1, uint32_t /*baud_rate*/ = 9600)
        : status(SensorStatus::Success), data{37.7749f, -122.4194f, 0.0f, 0.0f, 0.0f, 0, 0}
    {
    }

    virtual ~GpsSensor() = default;

    SensorStatus begin() override
    {
        return status;
    }

    SensorStatus getStatus() const override
    {
        return status;
    }

    const GPSData& read() override
    {
        return data;
    }

    void setData(const GPSData& Data)
    {
        data = Data;
    }

    void setStatus(SensorStatus newStatus)
    {
        status = newStatus;
    }
};

} // namespace HAL
