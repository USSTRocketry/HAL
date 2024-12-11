#pragma once

//SPI
#define SPI_MOSI         26
#define SPI_SCK          27
#define SPI_MISO         1

//I2C
#define I2C_SDA          18
#define I2C_SCL          19
#define I2C_WIRE         0  // Default Wire (SDA = 18, SCL = 19)

//LEDS
#define LED1             2
#define LED2             3
#define LED3             4

//GPS
#define GPS_TX           7
#define GPS_RX           8
#define GPS_EN           9
#define GPS_HW_SERIAL    2  // Using TX2 and RX2

//Adafruit RFM95 Radio
#define RADIO_CS         0
#define RADIO_RST        28
#define RADIO_EN         29
#define RADIO_G0         17
#define RADIO_G1         16
#define RADIO_G2         15
#define RADIO_G3         14
#define RADIO_G4         41
#define RADIO_G5         40

//RYLR998 UART radio
#define RADIO2_TX        34
#define RADIO2_RX        35
#define RADIO2_RST       36
#define RYLR998_HW_SERIAL   8  // Using TX8 and RX8
