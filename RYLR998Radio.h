#pragma once

#include "TelemetryRadio.h"
#include <HardwareSerial.h>  // For Teensy hardware serial

// Define macros for selecting hardware serial ports and corresponding pins
#define RYLR998_SERIAL0 1  // Serial1 (RX = 0, TX = 1)
#define RYLR998_SERIAL1 2  // Serial2 (RX = 7, TX = 8)
#define RYLR998_SERIAL2 3  // Serial3 (RX = 15, TX = 14)
#define RYLR998_SERIAL3 4  // Serial4 (RX = 16, TX = 17)
#define RYLR998_SERIAL4 5  // Serial5 (RX = 21, TX = 20)
#define RYLR998_SERIAL5 6  // Serial6 (RX = 25, TX = 24)
#define RYLR998_SERIAL6 7  // Serial7 (RX = 28, TX = 29)
#define RYLR998_SERIAL7 8  // Serial8 (RX = 34, TX = 35)

#define RYLR998_SERIAL(w) (w == 1 ? Serial1 \
                            : (w == 2 ? Serial2 \
                            : (w == 3 ? Serial3 \
                            : (w == 4 ? Serial4 \
                            : (w == 5 ? Serial5 \
                            : (w == 6 ? Serial6 \
                            : (w == 7 ? Serial7 \
                            : Serial8 \
                            )))))))

class RYLR998Radio : public TelemetryRadio {
private:
    HardwareSerial& rylr998;  // Reference to a hardware serial port
    uint32_t baudRate;

public:
    // Constructor accepts serial port as a parameter
    RYLR998Radio(uint8_t serialPort = RYLR998_SERIAL7, uint32_t baudRate = 9600);
    ~RYLR998Radio() override {}

    bool begin() override;
    bool send(const uint8_t* data, size_t length) override;
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    void setFrequency(float frequency) override;
    void setTxPower(uint8_t power) override;
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
};
