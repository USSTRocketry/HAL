#include "implementations/arduino/ArduinoRYLR998Radio.h"
#include <HardwareSerial.h>

// Macro for HardwareSerial mapping
#define HW_SERIAL(w) (w == 1 ? Serial1 \
                        : (w == 2 ? Serial2 \
                        : (w == 3 ? Serial3 \
                        : (w == 4 ? Serial4 \
                        : (w == 5 ? Serial5 \
                        : (w == 6 ? Serial6 \
                        : (w == 7 ? Serial7 \
                        : Serial8 \
                        )))))))

// Static instance of HardwareSerial
static HardwareSerial* rylr998_serial = &Serial1;

ArduinoRYLR998Radio::ArduinoRYLR998Radio(uint8_t serialPort, uint32_t baudRate)
    : serialPort(serialPort), baudRate(baudRate)
{
}

bool ArduinoRYLR998Radio::begin()
{
    // Begin the chosen serial port with the configured baud rate
    rylr998_serial->begin(baudRate);
    return true;
}

void ArduinoRYLR998Radio::reset(uint8_t resetPin)
{
    if (resetPin != 127) { // 127 is a sentinel value for "non-existent pin"
        pinMode(resetPin, OUTPUT);
        digitalWrite(resetPin, LOW);
        delay(10);
        digitalWrite(resetPin, HIGH);
        delay(10);
    }
}

bool ArduinoRYLR998Radio::send(const uint8_t* data, size_t length)
{
    // Send data using the selected hardware serial port
    for (size_t i = 0; i < length; ++i) {
        rylr998_serial->write(data[i]);
    }
    return true;
}

bool ArduinoRYLR998Radio::receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength)
{
    // Receive data using the selected hardware serial port
    receivedLength = 0;
    while (rylr998_serial->available() && receivedLength < maxLength) {
        buffer[receivedLength++] = rylr998_serial->read();
    }
    return receivedLength > 0;
}

void ArduinoRYLR998Radio::setFrequency(float frequency)
{
    // Configure frequency via AT command
    rylr998_serial->print("AT+FREQ=");
    rylr998_serial->print(static_cast<int>(frequency));
    rylr998_serial->print("\r\n");
}

void ArduinoRYLR998Radio::setTxPower(uint8_t power)
{
    // Configure TX power via AT command
    rylr998_serial->print("AT+POWER=");
    rylr998_serial->print(power);
    rylr998_serial->print("\r\n");
}

void ArduinoRYLR998Radio::configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate)
{
    // Configure LoRa parameters (if supported by RYLR998)
    // Note: RYLR998 may not expose direct access to SF, BW, CR through commands.
    rylr998_serial->print("AT+PARAMETER=");
    rylr998_serial->print(spreadingFactor);
    rylr998_serial->print(",");
    rylr998_serial->print(bandwidth);
    rylr998_serial->print(",");
    rylr998_serial->print(codingRate);
    rylr998_serial->print("\r\n");
}

void ArduinoRYLR998Radio::setAddress(const uint8_t address)
{
    // Configure address via AT command
    rylr998_serial->print("AT+ADDR=");
    rylr998_serial->print(address);
    rylr998_serial->print("\r\n");
}

void ArduinoRYLR998Radio::setDestinationAddress(const uint8_t address)
{
    // Set destination address for communication
    rylr998_serial->print("AT+DEST=");
    rylr998_serial->print(address);
    rylr998_serial->print("\r\n");
}

void ArduinoRYLR998Radio::setPromiscuousMode(bool enable)
{
    // Configure promiscuous mode if supported
    rylr998_serial->print("AT+PROMISC=");
    rylr998_serial->print(enable ? "1" : "0");
    rylr998_serial->print("\r\n");
}
