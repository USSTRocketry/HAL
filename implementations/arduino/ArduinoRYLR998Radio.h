#pragma once

#include "abstractions/ITelemetryRadio.h"
#include <cstdint>

/**
 * @brief Arduino-specific implementation of RYLR998 LoRa radio.
 * 
 * This implementation uses the RYLR998 LoRa module via serial communication.
 * For non-Arduino platforms, implement ITelemetryRadio directly.
 */
class ArduinoRYLR998Radio : public ITelemetryRadio
{
private:
    uint8_t serialPort;
    uint32_t baudRate;

public:
    /**
     * @brief Construct a RYLR998 radio interface.
     * @param serialPort Hardware serial port number for communication.
     * @param baudRate Baud rate for serial communication (default: 9600).
     */
    ArduinoRYLR998Radio(uint8_t serialPort, uint32_t baudRate = 9600);

    virtual ~ArduinoRYLR998Radio() override {}

    bool begin() override;
    void reset(uint8_t resetPin = 127) override;
    bool send(const uint8_t* data, size_t length) override;
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
    void setAddress(const uint8_t address) override;
    void setDestinationAddress(const uint8_t address) override;
    void setPromiscuousMode(bool enable) override;
};
