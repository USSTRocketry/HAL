#include "implementations/arduino/ArduinoRFM95Radio.h"
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>
#include <RHHardwareSPI2.h>

// Hardware SPI instances
extern RHHardwareSPI hardware_spi;
extern RHHardwareSPI1 hardware_spi1;
extern RHHardwareSPI2 hardware_spi2;

// Static instance of RH_RF95
static RH_RF95* rf95_instance = nullptr;

RHGenericSPI* getSPIInstance(uint8_t spiIndex) {
    if (spiIndex == 1) {
        return &hardware_spi1;
    } else if (spiIndex == 2) {
        return &hardware_spi2;
    }
    return &hardware_spi; // Default to SPI0
}

ArduinoRFM95Radio::ArduinoRFM95Radio(uint8_t csPin, uint8_t intPin, uint8_t spiIndex, float frequency)
    : csPin(csPin), intPin(intPin), spiIndex(spiIndex), frequency(frequency)
{
}

bool ArduinoRFM95Radio::begin()
{
    rf95_instance = new RH_RF95(csPin, intPin, *getSPIInstance(spiIndex));
    
    if (!rf95_instance->init()) {
        return false;
    }

    rf95_instance->setFrequency(frequency);
    rf95_instance->setTxPower(13, false); // Default TX power, 13 dBm
    return true;
}

void ArduinoRFM95Radio::reset(uint8_t resetPin)
{
    if (resetPin != 127) { // 127 is a sentinel value for "non-existent pin"
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, LOW);
        delay(10);
        digitalWrite(resetPin, HIGH);
        delay(10);
    }
}

bool ArduinoRFM95Radio::send(const uint8_t* data, size_t length)
{
    if (!rf95_instance) return false;
    return rf95_instance->send(data, length) && rf95_instance->waitPacketSent();
}

bool ArduinoRFM95Radio::receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength)
{
    if (!rf95_instance) return false;
    
    if (rf95_instance->available()) {
        uint8_t len = maxLength;
        if (rf95_instance->recv(buffer, &len)) {
            receivedLength = len;
            return true;
        }
    }
    return false;
}

void ArduinoRFM95Radio::setFrequency(float frequency)
{
    this->frequency = frequency;
    if (rf95_instance) {
        rf95_instance->setFrequency(frequency);
    }
}

void ArduinoRFM95Radio::setTxPower(uint8_t power)
{
    if (rf95_instance) {
        rf95_instance->setTxPower(power, false);
    }
}

void ArduinoRFM95Radio::configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate)
{
    // The RadioHead library does not provide direct access to these settings,
    // but they are typically configured at the register level in RH_RF95.
    // Example for manual register configuration:
    // rf95_instance->spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, (bandwidth << 4) | (codingRate << 1));
    // rf95_instance->spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, (spreadingFactor << 4) | 0x04);
}

void ArduinoRFM95Radio::setAddress(const uint8_t address)
{
    if (rf95_instance) {
        rf95_instance->setThisAddress(address);
        rf95_instance->setHeaderFrom(address);
    }
}

void ArduinoRFM95Radio::setDestinationAddress(const uint8_t address)
{
    if (rf95_instance) {
        rf95_instance->setHeaderTo(address);
    }
}

void ArduinoRFM95Radio::setPromiscuousMode(bool enable)
{
    if (rf95_instance) {
        rf95_instance->setPromiscuous(enable);
    }
}
