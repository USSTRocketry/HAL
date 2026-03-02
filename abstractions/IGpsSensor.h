#pragma once

#include "abstractions/ISensor.h"
#include "types.h"

namespace HAL {

/**
 * @brief Abstract interface for GPS sensor implementations.
 */
class IGpsSensor : public ISensor
{
public:
    virtual ~IGpsSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Reference to GPSData structure.
     */
    virtual const GPSData& read() = 0;
};

} // namespace HAL
