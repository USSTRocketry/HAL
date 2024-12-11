#include "SensorBMP280.h"

SensorBMP280::SensorBMP280(uint8_t i2c_addr, uint8_t i2c_wire, float ground_alt)
: Sensor(i2c_addr, i2c_wire)
, ground_alt(ground_alt)
{
    bmp = Adafruit_BMP280(&I2C_WIRE(i2c_wire));
}

SensorBMP280::SensorBMP280(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float ground_alt)
: Sensor(cs, miso, mosi, sck)
, ground_alt(ground_alt)
{
    bmp = Adafruit_BMP280(cs, mosi, miso, sck);
}

SensorBMP280::~SensorBMP280(){}

uint8_t SensorBMP280::begin()
{
    if(sensor_mode == SENSOR_MODE_I2C)
    {
        status = bmp.begin(i2c_addr);
    } else
    {
        status = bmp.begin();
    }

    if(status)
    {
        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }

    return status;
}

BMP280Data* SensorBMP280::read(){
    data.altitude       = bmp.readAltitude(ground_alt);
    data.pressure       = bmp.readPressure();
    data.temperature    = bmp.readTemperature();
    return &data;
}
