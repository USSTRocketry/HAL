#pragma once

#include "abstractions/ISensor.h"
#include "types.h"

/**
 * @brief Abstract interface for BMP280 barometric sensor implementations.
 */
class IBmp280Sensor : public ISensor
{
public:
    virtual ~IBmp280Sensor() = default;

    /**
     * @brief Read sensor data.
     * @return Pointer to BMP280Data structure.
     */
    virtual BMP280Data* read() = 0;
};
