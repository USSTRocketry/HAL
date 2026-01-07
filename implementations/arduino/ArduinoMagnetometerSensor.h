#pragma once

#include "abstractions/IMagnetometerSensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Arduino-specific implementation of Magnetometer sensor (LIS3MDL).
 * 
 * This implementation uses the Adafruit LIS3MDL library and Arduino I2C/SPI
 * interfaces. For non-Arduino platforms, implement IMagnetometerSensor directly.
 */
class ArduinoMagnetometerSensor : public IMagnetometerSensor
{
private:
    // I2C
    uint8_t i2c_addr;
    uint8_t i2c_wire;

    // SPI
    uint8_t spi_cs;
    uint8_t spi_miso;
    uint8_t spi_mosi;
    uint8_t spi_sck;

    uint8_t sensor_mode;
    uint8_t status;
    MagnetometerData data;

public:
    /**
     * @brief Construct a Magnetometer sensor for I2C communication.
     */
    ArduinoMagnetometerSensor(uint8_t i2c_addr, uint8_t i2c_wire);

    /**
     * @brief Construct a Magnetometer sensor for SPI communication.
     */
    ArduinoMagnetometerSensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);

    virtual ~ArduinoMagnetometerSensor();

    uint8_t begin() override;
    uint8_t getStatus() const override;
    MagnetometerData* read() override;
};
