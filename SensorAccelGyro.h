#include <Adafruit_LSM6DSOX.h>

#include "types.h"
#include "Sensor.h"


class SensorAccelGyro: Sensor
{
private:
    Adafruit_LSM6DSOX lsm6dsox;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

public:
    AccelGyroData data;

public:
    SensorAccelGyro(uint8_t i2c_addr = LSM6DS_I2CADDR_DEFAULT, uint8_t i2c_wire = I2C_WIRE0);
    SensorAccelGyro(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
    uint8_t begin();
    AccelGyroData* read();
    ~SensorAccelGyro();
};