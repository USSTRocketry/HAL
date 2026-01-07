#pragma once

#include <cstdint>
#include <string>

class ITelemetryRadio {
public:
    virtual ~ITelemetryRadio() {}

    // Initialize the radio with default or customized settings
    virtual bool begin() = 0;

    virtual void reset(uint8_t resetPin = 127) = 0; // Non esistent pin

    // Send data over the radio
    virtual bool send(const uint8_t* data, size_t length) = 0;

    // Receive data from the radio
    virtual bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) = 0;

    // Set a custom frequency
    virtual void setFrequency(float frequency) = 0;

    // Set a custom transmission power
    virtual void setTxPower(uint8_t power) = 0;

    // Configure spreading factor, bandwidth, etc. (LoRa-specific settings)
    virtual void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) = 0;

    // Set radio adress
    virtual void setAddress(const uint8_t address) = 0;

    // set destination address
    virtual void setDestinationAddress(const uint8_t address) = 0;

    // Set promiscuous mode for receiving all packets (Only for LoRa)
    virtual void setPromiscuousMode(bool enable) = 0;
};
