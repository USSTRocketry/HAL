#pragma once

#include "TelemetryRadio.h"
#include <RH_RF95.h>

class RFM95Radio : public TelemetryRadio {
private:
    RH_RF95 rf95;
    uint8_t csPin;
    uint8_t intPin;
    float frequency;

public:
    RFM95Radio(uint8_t csPin, uint8_t intPin, float frequency = 915.0);
    ~RFM95Radio() override {}

    bool begin() override;
    bool send(const uint8_t* data, size_t length) override;
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
};
