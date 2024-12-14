#pragma once

#include "TelemetryRadio.h"
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>
#include <RHHardwareSPI2.h>
#include "PINS.h"
#include "types.h"


class RFM95Radio : public TelemetryRadio {
private:
    RH_RF95 rf95;
    uint8_t csPin;
    uint8_t intPin;
    float frequency;
    uint8_t spiIndex;

    RHGenericSPI* getSPIInstance(uint8_t spiIndex);

public:
    RFM95Radio(uint8_t csPin, uint8_t intPin, uint8_t spiIndex = HW_SPI0, float frequency = 915.0);
    ~RFM95Radio() override {}

    bool begin() override;
    bool send(const uint8_t* data, size_t length) override;
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
};
