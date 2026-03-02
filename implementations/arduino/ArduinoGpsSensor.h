#pragma once

#include "abstractions/IGpsSensor.h"
#include "types.h"
#include <cstdint>

namespace HAL {

/**
 * @brief Arduino-specific implementation of GPS sensor (Adafruit GPS).
 * 
 * This implementation uses the Adafruit GPS library and Arduino HardwareSerial
 * interface. For non-Arduino platforms, implement IGpsSensor directly.
 */
class GpsSensor : public IGpsSensor
{
private:
    uint8_t serial_port;
    uint32_t baud_rate;
    SensorStatus status;
    GPSData data;

public:
    /**
     * @brief Construct a GPS sensor for serial communication.
     */
    GpsSensor(uint8_t serial_port, uint32_t baud_rate = 9600);

    virtual ~GpsSensor();

    SensorStatus begin() override;
    SensorStatus getStatus() const override;
    const GPSData& read() override;

    /**
     * @brief Configure GPS settings.
     */
    void configure(uint32_t update_rate_ms = 1000, const char* output_mode = nullptr);

    /**
     * @brief Update GPS with new serial data.
     * Should be called frequently to process incoming GPS data.
     */
    void update();

    /**
     * @brief Check if GPS has a fix.
     */
    bool hasFix();

    /**
     * @brief Send a raw command to the GPS module.
     */
    void sendCommand(const char* command);
};

} // namespace HAL
