#pragma once

typedef struct _bmp280Data
{
    float temperature;
    float pressure;
    float altitude;
} BMP280Data;

typedef struct _accelGyroData
{
    float temperature;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
} AccelGyroData;

typedef struct _magnetometerData
{
    float magneticX;
    float magneticY;
    float magneticZ;
} MagnetometerData;

typedef struct _temperatureData
{
    float temperature;
} TemperatureData;

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;       // in knots
    float angle;       // course over ground
    uint8_t satellites;
    uint8_t fix_quality;  // GPS signal quality
} GPSData;

