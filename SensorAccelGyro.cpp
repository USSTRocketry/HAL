#include "SensorAccelGyro.h"


SensorAccelGyro::SensorAccelGyro(uint8_t i2c_addr, uint8_t i2c_wire)
: Sensor(i2c_addr, i2c_wire)
{}

SensorAccelGyro::SensorAccelGyro(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
: Sensor(cs, miso, mosi, sck)
{}

SensorAccelGyro::~SensorAccelGyro(){}

uint8_t SensorAccelGyro::begin()
{
    if(sensor_mode == SENSOR_MODE_I2C)
    {
        status = lsm6dsox.begin_I2C(i2c_addr, &I2C_WIRE(i2c_wire), 0);
    } else
    {
        status = lsm6dsox.begin_SPI(spi_cs, spi_sck, spi_miso, spi_mosi);
    }

    return status;
}

AccelGyroData* SensorAccelGyro::read(){
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    data.temperature    = temp.temperature;
    data.accelX         = accel.acceleration.x;
    data.accelY         = accel.acceleration.y;
    data.accelZ         = accel.acceleration.z;
    data.gyroX          = gyro.gyro.x;
    data.gyroY          = gyro.gyro.y;
    data.gyroZ          = gyro.gyro.z;
    return &data;
}
