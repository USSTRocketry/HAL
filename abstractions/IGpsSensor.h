#pragma once

#include "abstractions/ISensor.h"
#include "types.h"

/**
 * @brief Abstract interface for GPS sensor implementations.
 */
class IGpsSensor : public ISensor
{
public:
    virtual ~IGpsSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Pointer to GPSData structure.
     */
    virtual GPSData* read() = 0;
};
