#pragma once

#include "abstractions/IBmp280Sensor.h"
#include "types.h"
#include <cstdint>

namespace HAL {

/**
 * @brief Arduino-specific implementation of BMP280 barometric sensor.
 * 
 * This implementation uses the Adafruit BMP280 library and Arduino I2C/SPI
 * interfaces. For non-Arduino platforms, implement IBmp280Sensor directly.
 */
class Bmp280Sensor : public IBmp280Sensor
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
    SensorStatus status;
    float sea_level_hpa;
    BMP280Data data;

    // Forward declarations - will be defined in Arduino implementation
    void initializeI2C();
    void initializeSPI();

public:
    /**
     * @brief Construct a BMP280 sensor for I2C communication.
     */
    Bmp280Sensor(uint8_t i2c_addr, uint8_t i2c_wire, float sea_level_hpa = 0.0f);

    /**
     * @brief Construct a BMP280 sensor for SPI communication.
     */
    Bmp280Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float sea_level_hpa = 0.0f);

    virtual ~Bmp280Sensor();

    SensorStatus begin() override;
    SensorStatus getStatus() const override;
    const BMP280Data& read() override;
};

} // namespace HAL
