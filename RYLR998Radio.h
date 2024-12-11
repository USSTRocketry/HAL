#pragma once

#include "TelemetryRadio.h"
#include <SoftwareSerial.h> // UART communication for RYLR998

class RYLR998Radio : public TelemetryRadio {
private:
    SoftwareSerial rylr998;
    uint32_t baudRate;

public:
    RYLR998Radio(uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 9600);
    ~RYLR998Radio() override {}

    bool begin() override;
    bool send(const uint8_t* data, size_t length) override;
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
};
