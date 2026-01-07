#pragma once

#include "abstractions/ISensor.h"
#include "types.h"

/**
 * @brief Abstract interface for temperature sensor implementations.
 */
class ITemperatureSensor : public ISensor
{
public:
    virtual ~ITemperatureSensor() = default;

    /**
     * @brief Read sensor data.
     * @return Pointer to TemperatureData structure.
     */
    virtual TemperatureData* read() = 0;
};
