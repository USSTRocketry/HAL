#pragma once

#include "abstractions/ITelemetryRadio.h"
#include <cstdint>
#include <span>

namespace HAL {

/**
 * @brief Arduino-specific implementation of RYLR998 LoRa radio.
 * 
 * This implementation uses the RYLR998 LoRa module via serial communication.
 * For non-Arduino platforms, implement ITelemetryRadio directly.
 */
class RYLR998Radio : public ITelemetryRadio
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
    RYLR998Radio(uint8_t serialPort, uint32_t baudRate = 9600);

    virtual ~RYLR998Radio() override {}

    bool begin() override;
    void reset(uint8_t resetPin = 127) override;
    bool send(std::span<const uint8_t> data) override;
    bool receive(std::span<uint8_t> buffer, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
    void setAddress(const uint8_t address) override;
    void setDestinationAddress(const uint8_t address) override;
    void setPromiscuousMode(bool enable) override;
    void* native_handle() override;
};

} // namespace HAL
