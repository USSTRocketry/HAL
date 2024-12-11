#include "RYLR998Radio.h"

RYLR998Radio::RYLR998Radio(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
    : rylr998(rxPin, txPin), baudRate(baudRate) {}

bool RYLR998Radio::begin() {
    rylr998.begin(baudRate);
    rylr998.print("AT+ADDRESS=0\r\n"); // Example: Set default address
    return true;
}

bool RYLR998Radio::send(const uint8_t* data, size_t length) {
    rylr998.print("AT+SEND=");
    rylr998.print(length);
    rylr998.print(",");
    rylr998.write(data, length);
    rylr998.print("\r\n");
    return true;
}

bool RYLR998Radio::receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) {
    receivedLength = 0;
    while (rylr998.available() && receivedLength < maxLength) {
        buffer[receivedLength++] = rylr998.read();
    }
    return receivedLength > 0;
}

void RYLR998Radio::setFrequency(float frequency) {
    // Configure frequency via AT command
    rylr998.print("AT+FREQ=");
    rylr998.print(static_cast<int>(frequency)); // Convert to integer for command
    rylr998.print("\r\n");
}

void RYLR998Radio::setTxPower(uint8_t power) {
    // Configure TX power via AT command
    rylr998.print("AT+POWER=");
    rylr998.print(power);
    rylr998.print("\r\n");
}

void RYLR998Radio::configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) {
    // Configure LoRa parameters (if supported by RYLR998)
    // Note: RYLR998 may not expose direct access to SF, BW, CR through commands.
    rylr998.print("AT+PARAMETER=");
    rylr998.print(spreadingFactor);
    rylr998.print(",");
    rylr998.print(bandwidth);
    rylr998.print(",");
    rylr998.print(codingRate);
    rylr998.print("\r\n");
}
