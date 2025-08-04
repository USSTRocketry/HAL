#pragma once

#include "TelemetryRadio.h"
#include <RH_RF95.h>
#include <RHHardwareSPI1.h>
#include <RHHardwareSPI2.h>
#include "PINS.h"
#include "types.h"

/**
 * @class RFM95Radio
 * @brief Represents a telemetry radio interface using the RFM95 (915MHz and 433MHz) LoRa module.
 *
 */
class RFM95Radio : public TelemetryRadio {
private:
    RH_RF95 rf95;          ///< Instance of RH_RF95 for LoRa communication
    uint8_t csPin;         ///< Chip Select (CS) pin for SPI communication
    uint8_t intPin;        ///< Interrupt pin for RFM95
    float frequency;       ///< Frequency of the RFM95 in MHz
    uint8_t spiIndex;      ///< SPI interface index (0->SPI0, 1->SPI1, or 2->SPI2)

    /**
     * @brief Get the RHGenericSPI instance corresponding to the SPI index.
     * @param spiIndex The index of the SPI interface (e.g., HW_SPI0, HW_SPI1, HW_SPI2).
     * @return Pointer to the corresponding RHGenericSPI instance.
     */
    RHGenericSPI* getSPIInstance(uint8_t spiIndex);

public:
    /**
     * @brief               Constructor for the RFM95Radio class.
     * @param csPin         Chip Select pin for the RFM95 module.
     * @param intPin        Interrupt pin connected to the RFM95 DIO0 line.
     * @param spiIndex      SPI interface index (default: HW_SPI0).
     * @param frequency     Frequency for LoRa communication in MHz (default: 915.0).
     */
    RFM95Radio(uint8_t csPin, uint8_t intPin, uint8_t spiIndex = HW_SPI1, float frequency = 915.0);

    /**
     * @brief Destructor for the RFM95Radio class.
     */
    ~RFM95Radio() override {}

    /**
     * @brief Initialize the RFM95 module.
     * @return true if initialization succeeds, false otherwise.
     */
    bool begin() override;

    /**
     * @brief Reset the RFM95 module.
     */
    void reset(uint8_t resetPin) override;


    /**
     * @brief Send data using the RFM95 module.
     * @param data Pointer to the data to send.
     * @param length Length of the data in bytes.
     * @return true if the data is sent successfully, false otherwise.
     */
    bool send(const uint8_t* data, size_t length) override;

    /**
     * @brief Receive data using the RFM95 module.
     * @param buffer Pointer to the buffer to store received data.
     * @param maxLength Maximum length of the buffer.
     * @param receivedLength Reference to store the actual received length.
     * @return true if data is received successfully, false otherwise.
     */
    bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength) override;

    /**
     * @brief Set the frequency for LoRa communication.
     * @param frequency Frequency in MHz to set.
     */
    void setFrequency(float frequency) override;

    /**
     * @brief Set the transmission power for the RFM95 module.
     * @param power Transmission power in dBm (typical range: 5-20).
     */
    void setTxPower(uint8_t power) override;

    /**
     * @brief Configure LoRa parameters such as spreading factor, bandwidth, and coding rate.
     * @param spreadingFactor Spreading factor (e.g., 7-12).
     * @param bandwidth Bandwidth in kHz (e.g., 125, 250, 500).
     * @param codingRate Coding rate (e.g., 5 for 4/5, 6 for 4/6).
     */
    void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate) override;

    /**
     * @brief Set the radio address for the RFM95 module.
     * @param address Address to set (0-255).
     */
    void setAddress(const uint8_t address) override;

    /**
     * @brief Set the destination address for the RFM95 module.
     * @param address Destination address to set (0-255).
     */
    void setDestinationAddress(const uint8_t address) override;

    /**
     * @brief Set promiscuous mode for receiving all packets.
     * @param enable true to enable promiscuous mode, false to disable.
     */
    void setPromiscuousMode(bool enable) override;
};
