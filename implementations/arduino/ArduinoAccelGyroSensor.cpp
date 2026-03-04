#include "ArduinoAccelGyroSensor.h"
#include <Adafruit_LSM6DSOX.h>

// Macro definitions from Sensor.h
#define I2C_WIRE0   0 // default (SDA = 18, SCL = 19)
#define I2C_WIRE1   1 // (SDA = 17, SCL = 16)
#define I2C_WIRE2   2 // (SDA = 25, SCL = 24)

#define MAP_I2C_WIRE(w) (w == 0 ? Wire : (w == 1 ? Wire1 : Wire2))

#define SENSOR_MODE_I2C 0
#define SENSOR_MODE_SPI 1

namespace HAL {

// Static instance of Adafruit_LSM6DSOX
static Adafruit_LSM6DSOX lsm6dsox;
static sensors_event_t accel;
static sensors_event_t gyro;
static sensors_event_t temp;

AccelGyroSensor::AccelGyroSensor(uint8_t i2c_addr, uint8_t i2c_wire)
    : i2c_addr(i2c_addr), i2c_wire(i2c_wire), sensor_mode(SENSOR_MODE_I2C), status(SensorStatus::Failure)
{
}

AccelGyroSensor::AccelGyroSensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
    : spi_cs(cs), spi_miso(miso), spi_mosi(mosi), spi_sck(sck), sensor_mode(SENSOR_MODE_SPI), status(SensorStatus::Failure)
{
}

AccelGyroSensor::~AccelGyroSensor() {}

SensorStatus AccelGyroSensor::begin()
{
    bool initSuccess = false;
    if (sensor_mode == SENSOR_MODE_I2C) {
        // Note: LSM6DS_I2CADDR_DEFAULT is typically 0x6A
        initSuccess = lsm6dsox.begin_I2C(i2c_addr, &MAP_I2C_WIRE(i2c_wire), 0);
    } else {
        initSuccess = lsm6dsox.begin_SPI(spi_cs, spi_sck, spi_miso, spi_mosi);
    }

    status = initSuccess ? SensorStatus::Success : SensorStatus::Failure;
    return status;
}

SensorStatus AccelGyroSensor::getStatus() const
{
    return status;
}

const AccelGyroData& AccelGyroSensor::read()
{
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    data.temperature = temp.temperature;
    data.accelX = accel.acceleration.x;
    data.accelY = accel.acceleration.y;
    data.accelZ = accel.acceleration.z;
    data.gyroX = gyro.gyro.x;
    data.gyroY = gyro.gyro.y;
    data.gyroZ = gyro.gyro.z;
    return data;
}

} // namespace HAL
