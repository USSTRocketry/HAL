#pragma once

#include "ISensor.h"
#include "types.h"

namespace HAL {

/**
 * @brief Abstract interface for accelerometer/gyroscope sensor implementations.
 */
class IAccelGyroSensor : public ISensor
{
public:
    virtual ~IAccelGyroSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Reference to AccelGyroData structure.
     */
    virtual const AccelGyroData& read() = 0;
};

} // namespace HAL
