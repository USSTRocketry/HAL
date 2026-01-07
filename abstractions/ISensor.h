#pragma once

#include <cstdint>

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
     * @return Non-zero on success, 0 on failure.
     */
    virtual uint8_t begin() = 0;

    /**
     * @brief Get the initialization status of the sensor.
     * @return Non-zero if initialized, 0 otherwise.
     */
    virtual uint8_t getStatus() const = 0;
};
