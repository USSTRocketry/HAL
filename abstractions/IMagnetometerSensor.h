#pragma once

#include "ISensor.h"
#include "types.h"

namespace HAL {

/**
 * @brief Abstract interface for magnetometer sensor implementations.
 */
class IMagnetometerSensor : public ISensor
{
public:
    virtual ~IMagnetometerSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Reference to MagnetometerData structure.
     */
    virtual const MagnetometerData& read() = 0;
};

} // namespace HAL
