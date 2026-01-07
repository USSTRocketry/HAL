#pragma once

#include "abstractions/ISensor.h"
#include "types.h"

/**
 * @brief Abstract interface for accelerometer/gyroscope sensor implementations.
 */
class IAccelGyroSensor : public ISensor
{
public:
    virtual ~IAccelGyroSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Pointer to AccelGyroData structure.
     */
    virtual AccelGyroData* read() = 0;
};
