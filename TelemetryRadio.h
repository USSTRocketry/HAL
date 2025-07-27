#pragma once

#include <cstdint>
#include <string>

class TelemetryRadio
{
public:
    virtual ~TelemetryRadio() {}

    // Initialize the radio with default or customized settings
    virtual bool begin() = 0;

    virtual void reset(uint8_t resetPin = 127) = 0; // Non esistent pin

    // Send data over the radio
    virtual bool send(const uint8_t* data, size_t length) = 0;

    // Receive data from the radio
    virtual std::pair<bool, size_t> receive(uint8_t* buffer, size_t maxLength) = 0;

    // Set a custom frequency
    virtual void setFrequency(float frequency) = 0;

    // Set a custom transmission power
    virtual void setTxPower(uint8_t power) = 0;

    // Configure spreading factor, bandwidth, etc. (LoRa-specific settings)
    virtual void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) = 0;
};
