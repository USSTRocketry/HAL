#pragma once

#include "ISensor.h"
#include "types.h"

namespace HAL {

/**
 * @brief Abstract interface for temperature sensor implementations.
 */
class ITemperatureSensor : public ISensor
{
public:
    virtual ~ITemperatureSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Reference to TemperatureData structure.
     */
    virtual const TemperatureData& read() = 0;
};

} // namespace HAL
