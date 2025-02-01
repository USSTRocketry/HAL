#pragma once

// SPI
constexpr auto SPI_MOSI = 26;
constexpr auto SPI_SCK  = 27;
constexpr auto SPI_MISO = 1;

// I2C
constexpr auto I2C_SDA  = 18;
constexpr auto I2C_SCL  = 19;
constexpr auto I2C_WIRE = 0; // Default Wire (SDA = 18, SCL = 19)

// LEDS
constexpr auto LED1 = 2;
constexpr auto LED2 = 3;
constexpr auto LED3 = 4;

// GPS
constexpr auto GPS_TX        = 7;
constexpr auto GPS_RX        = 8;
constexpr auto GPS_EN        = 9;
constexpr auto GPS_HW_SERIAL = 2; // Using TX2 and RX2

// Adafruit RFM95 Radio
constexpr auto RADIO_CS  = 0;
constexpr auto RADIO_RST = 28;
constexpr auto RADIO_EN  = 29;
constexpr auto RADIO_G0  = 17;
constexpr auto RADIO_G1  = 16;
constexpr auto RADIO_G2  = 15;
constexpr auto RADIO_G3  = 14;
constexpr auto RADIO_G4  = 41;
constexpr auto RADIO_G5  = 40;
constexpr auto RADIO_INT = RADIO_G0; // Interrupt pin

// RYLR998 UART radio
constexpr auto RADIO2_TX         = 34;
constexpr auto RADIO2_RX         = 35;
constexpr auto RADIO2_RST        = 36;
constexpr auto RYLR998_HW_SERIAL = 8; // Using TX8 and RX8
