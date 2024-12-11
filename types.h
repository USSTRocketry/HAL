#pragma once

// Define macros for selecting hardware serial ports and corresponding pins
#define HW_SERIAL1 1  // Serial1 (RX = 0, TX = 1)
#define HW_SERIAL2 2  // Serial2 (RX = 7, TX = 8)
#define HW_SERIAL3 3  // Serial3 (RX = 15, TX = 14)
#define HW_SERIAL4 4  // Serial4 (RX = 16, TX = 17)
#define HW_SERIAL5 5  // Serial5 (RX = 21, TX = 20)
#define HW_SERIAL6 6  // Serial6 (RX = 25, TX = 24)
#define HW_SERIAL7 7  // Serial7 (RX = 28, TX = 29)
#define HW_SERIAL8 8  // Serial8 (RX = 34, TX = 35)

#define HW_SERIAL(w) (w == 1 ? Serial1 \
                        : (w == 2 ? Serial2 \
                        : (w == 3 ? Serial3 \
                        : (w == 4 ? Serial4 \
                        : (w == 5 ? Serial5 \
                        : (w == 6 ? Serial6 \
                        : (w == 7 ? Serial7 \
                        : Serial8 \
                        )))))))

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

