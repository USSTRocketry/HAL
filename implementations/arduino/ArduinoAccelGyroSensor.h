#pragma once

#include "abstractions/IAccelGyroSensor.h"
#include "types.h"
#include <cstdint>

/**
 * @brief Arduino-specific implementation of AccelGyro sensor (LSM6DSOX).
 * 
 * This implementation uses the Adafruit LSM6DSOX library and Arduino I2C/SPI
 * interfaces. For non-Arduino platforms, implement IAccelGyroSensor directly.
 */
class ArduinoAccelGyroSensor : public IAccelGyroSensor
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
    AccelGyroData data;

public:
    /**
     * @brief Construct an AccelGyro sensor for I2C communication.
     */
    ArduinoAccelGyroSensor(uint8_t i2c_addr, uint8_t i2c_wire);

    /**
     * @brief Construct an AccelGyro sensor for SPI communication.
     */
    ArduinoAccelGyroSensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);

    virtual ~ArduinoAccelGyroSensor();

    uint8_t begin() override;
    uint8_t getStatus() const override;
    AccelGyroData* read() override;
};
