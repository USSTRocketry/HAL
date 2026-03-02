#pragma once

#include "types.h"

namespace HAL {

/**
 * @brief Abstract base class for all sensor implementations.
 * 
 * This interface defines the contract for all sensor implementations,
 * allowing different hardware backends (Arduino, STM32, etc.) to be
 * swapped without changing the HAL API or client code.
 */
class ISensor
{
public:
    virtual ~ISensor() = default;

    /**
     * @brief Initialize the sensor.
     * @return SensorStatus::Success on success, SensorStatus::Failure on failure.
     */
    virtual SensorStatus begin() = 0;

    /**
     * @brief Get the initialization status of the sensor.
     * @return SensorStatus::Success if initialized, SensorStatus::Failure otherwise.
     */
    virtual SensorStatus getStatus() const = 0;
};

} // namespace HAL
