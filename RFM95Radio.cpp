#include "RFM95Radio.h"

RFM95Radio::RFM95Radio(uint8_t csPin, uint8_t intPin, uint8_t spiIndex, float frequency) :
    rf95(csPin, intPin, getSPIInstance(spiIndex)),
    csPin(csPin),
    intPin(intPin),
    frequency(frequency),
    spiIndex(spiIndex)
{
}

RHGenericSPI& RFM95Radio::getSPIInstance(uint8_t spiIndex)
{
    if (spiIndex == HW_SPI1)
    {
        return &hardware_spi1;
    }
    else if (spiIndex == HW_SPI2)
    {
        return &hardware_spi2;
    }
    return &hardware_spi; // Default to SPI0
}

bool RFM95Radio::begin()
{
    if (!rf95.init())
    {
        return false;
    }

    rf95.setFrequency(frequency);
    rf95.setTxPower(13, false); // Default TX power, 13 dBm
    return true;
}

void RFM95Radio::reset(uint8_t resetPin) {
    pinMode(resetPin, OUTPUT);
    digitalWrite(resetPin, LOW);
    delay(10); // Wait for 10 ms
    digitalWrite(resetPin, HIGH);
    delay(10); // Wait for 10 ms
}

bool RFM95Radio::send(const uint8_t* data, size_t length) {
    return rf95.send(data, length) && rf95.waitPacketSent();
}

std::pair<bool, size_t> RFM95Radio::receive(uint8_t* const buffer, size_t maxLength)
{
    if (rf95.available())
    {
        uint8_t len = maxLength;
        if (rf95.recv(buffer, &len))
        {
            receivedLength = len;
            return true;
        }
    }
    return false;
}

void RFM95Radio::setFrequency(float frequency)
{
    this->frequency = frequency;
    rf95.setFrequency(frequency);
}

void RFM95Radio::setTxPower(uint8_t power)
{
    rf95.setTxPower(power, false); // Use false for high-power module handling
}

void RFM95Radio::configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate)
{
    // The RadioHead library does not provide direct access to these settings,
    // but they are typically configured at the register level in RH_RF95.
    // Example for manual register configuration:
    // rf95.spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, (bandwidth << 4) | (codingRate << 1));
    // rf95.spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, (spreadingFactor << 4) | 0x04);
}
