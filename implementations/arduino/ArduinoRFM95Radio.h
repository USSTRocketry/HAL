#pragma once

#include "abstractions/ITelemetryRadio.h"
#include <cstdint>

/**
 * @brief Arduino-specific implementation of RFM95 LoRa radio.
 * 
 * This implementation uses the RadioHead RH_RF95 library and Arduino SPI
 * interfaces. For non-Arduino platforms, implement ITelemetryRadio directly.
 */
class ArduinoRFM95Radio : public ITelemetryRadio
{
private:
    uint8_t csPin;
    uint8_t intPin;
    uint8_t spiIndex;
    float frequency;

public:
    /**
     * @brief Construct an RFM95 radio interface.
     * @param csPin Chip Select pin for the RFM95 module.
     * @param intPin Interrupt pin connected to the RFM95 DIO0 line.
     * @param spiIndex SPI interface index (default: HW_SPI1).
     * @param frequency Frequency for LoRa communication in MHz (default: 915.0).
     */
    ArduinoRFM95Radio(uint8_t csPin, uint8_t intPin, uint8_t spiIndex = 1, float frequency = 915.0);

    virtual ~ArduinoRFM95Radio() override {}

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
