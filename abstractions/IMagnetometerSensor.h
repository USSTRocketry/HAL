#pragma once

#include "abstractions/ISensor.h"
#include "types.h"

/**
 * @brief Abstract interface for magnetometer sensor implementations.
 */
class IMagnetometerSensor : public ISensor
{
public:
    virtual ~IMagnetometerSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Pointer to MagnetometerData structure.
     */
    virtual MagnetometerData* read() = 0;
};
