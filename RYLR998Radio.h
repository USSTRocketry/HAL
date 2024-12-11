#pragma once
#include "types.h"
#include "PINS.h"
#include "TelemetryRadio.h"
#include <HardwareSerial.h>  // For Teensy hardware serial

class RYLR998Radio : public TelemetryRadio {
private:
    HardwareSerial& rylr998;  // Reference to a hardware serial port
    uint32_t baudRate;

public:
    // Constructor accepts serial port as a parameter
    RYLR998Radio(uint8_t serialPort = RYLR998_HW_SERIAL, uint32_t baudRate = 9600);
    ~RYLR998Radio() override {}

    bool begin() override;
    bool send(const uint8_t* data, size_t length) override;
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
};
