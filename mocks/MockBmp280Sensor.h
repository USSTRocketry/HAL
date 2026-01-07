#pragma once

#include "abstractions/IBmp280Sensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Mock implementation of BMP280 sensor for testing purposes.
 * 
 * This is a simple mock that can be used in unit tests.
 */
class MockBmp280Sensor : public IBmp280Sensor
{
private:
    BMP280Data data;
    uint8_t status;

public:
    MockBmp280Sensor()
        : status(1), data{20.5f, 101325.0f, 100.0f}
    {
    }

    virtual ~MockBmp280Sensor() = default;

    uint8_t begin() override
    {
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    BMP280Data* read() override
    {
        return &data;
    }

    // Allow tests to set mock data
    void setMockData(float temperature, float pressure, float altitude)
    {
        data.temperature = temperature;
        data.pressure = pressure;
        data.altitude = altitude;
    }

    void setStatus(uint8_t newStatus)
    {
        status = newStatus;
    }
};
