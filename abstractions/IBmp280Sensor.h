#pragma once

#include "ISensor.h"
#include "types.h"

namespace HAL {

/**
 * @brief Abstract interface for BMP280 barometric sensor implementations.
 */
class IBmp280Sensor : public ISensor
{
public:
    virtual ~IBmp280Sensor() = default;

    /**
     * @brief Read sensor data.
     * @return Reference to BMP280Data structure.
     */
    virtual const BMP280Data& read() = 0;
};

} // namespace HAL
