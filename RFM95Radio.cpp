#include "RFM95Radio.h"

RFM95Radio::RFM95Radio(uint8_t csPin, uint8_t intPin, float frequency)
    : rf95(csPin, intPin), csPin(csPin), intPin(intPin), frequency(frequency) {}

bool RFM95Radio::begin() {
    if (!rf95.init()) {
        return false;
    }

    rf95.setFrequency(frequency);
    rf95.setTxPower(13, false); // Default TX power, 13 dBm
    return true;
}

bool RFM95Radio::send(const uint8_t* data, size_t length) {
    return rf95.send(data, length) && rf95.waitPacketSent();
}

bool RFM95Radio::receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) {
    if (rf95.available()) {
        uint8_t len = maxLength;
        if (rf95.recv(buffer, &len)) {
            receivedLength = len;
            return true;
        }
    }
    return false;
}

void RFM95Radio::setFrequency(float frequency) {
    this->frequency = frequency;
    rf95.setFrequency(frequency);
}

void RFM95Radio::setTxPower(uint8_t power) {
    rf95.setTxPower(power, false); // Use false for high-power module handling
}

void RFM95Radio::configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) {
    // The RadioHead library does not provide direct access to these settings,
    // but they are typically configured at the register level in RH_RF95.
    // Example for manual register configuration (if needed):
    // rf95.spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, (bandwidth << 4) | (codingRate << 1));
    // rf95.spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, (spreadingFactor << 4) | 0x04);
}