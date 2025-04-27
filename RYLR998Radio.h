#pragma once
#include "types.h"
#include "PINS.h"
#include "TelemetryRadio.h"
#include <HardwareSerial.h>  // For Teensy hardware serial

/**
 * @class RYLR998Radio
 * @brief A class to interface with the RYLR998 LoRa radio module.
 * 
 * This class provides methods to initialize and communicate with the RYLR998 LoRa radio module.
 * It allows sending and receiving data, setting frequency, transmission power, and configuring
 * LoRa parameters such as spreading factor, bandwidth, and coding rate.
 * @note This class inherits from the TelemetryRadio base class.
 * 
 * @param rylr998 Reference to a hardware serial port used for communication with the RYLR998 module.
 * @param baudRate The baud rate for serial communication. Default is 9600.
 */
class RYLR998Radio : public TelemetryRadio {
private:
    HardwareSerial& rylr998;  // Reference to a hardware serial port
    uint32_t baudRate;

public:
    // Constructor accepts serial port as a parameter
    RYLR998Radio(uint8_t serialPort = RYLR998_HW_SERIAL, uint32_t baudRate = 9600);
    ~RYLR998Radio() override {}

    /**
     * @brief Sends data using the RYLR998 radio module.
     * @param data Pointer to the data to be sent.
     * @param length Length of the data to be sent.
     * @return true if the data was sent successfully, false otherwise.
     */
    bool send(const uint8_t* data, size_t length) override;
    /**
     * @brief Receives data from the RYLR998 radio module.
     * @param buffer Pointer to the buffer where received data will be stored.
     * @param maxLength Maximum length of data to be received.
     * @param receivedLength Reference to a variable where the actual length of received data will be stored.
     * @return true if data is received successfully, false otherwise.
     */
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;
    /**
     * @brief Initializes the RYLR998 radio module.
     * This method sets up the serial communication and configures the RYLR998 module for operation.
     * @return true if initialization is successful, false otherwise.
     */
    bool begin() override;
    /**
     * @brief Sets the frequency for the RYLR998 radio module.
     * @param frequency The frequency to be set.
     */
    void setFrequency(float frequency) override;
    /**
     * @brief Sets the transmission power for the RYLR998 radio module.
     * @param power The transmission power to be set (0-22 dBm).
     */
    void setTxPower(uint8_t power) override;
    /**
     * @brief Configures LoRa parameters for the RYLR998 radio module.
     * @param spreadingFactor The spreading factor to be set.
     * @param bandwidth The bandwidth to be set.
     * @param codingRate The coding rate to be set.
     */
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;
};
