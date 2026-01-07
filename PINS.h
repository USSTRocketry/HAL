#pragma once

#include <cstdint>

// Hardware serial identifiers
constexpr uint8_t HW_SERIAL1 = 1;  // Serial1 (RX = 0, TX = 1)
constexpr uint8_t HW_SERIAL2 = 2;  // Serial2 (RX = 7, TX = 8)
constexpr uint8_t HW_SERIAL3 = 3;  // Serial3 (RX = 15, TX = 14)
constexpr uint8_t HW_SERIAL4 = 4;  // Serial4 (RX = 16, TX = 17)
constexpr uint8_t HW_SERIAL5 = 5;  // Serial5 (RX = 21, TX = 20)
constexpr uint8_t HW_SERIAL6 = 6;  // Serial6 (RX = 25, TX = 24)
constexpr uint8_t HW_SERIAL7 = 7;  // Serial7 (RX = 28, TX = 29)
constexpr uint8_t HW_SERIAL8 = 8;  // Serial8 (RX = 34, TX = 35)

// SPI bus identifiers
constexpr uint8_t HW_SPI0 = 0; // hardware_spi
constexpr uint8_t HW_SPI1 = 1; // hardware_spi1
constexpr uint8_t HW_SPI2 = 2; // hardware_spi2


// SPI
constexpr uint8_t SPI_MOSI = 26;
constexpr uint8_t SPI_SCK  = 27;
constexpr uint8_t SPI_MISO = 1;

// I2C
constexpr uint8_t I2C_SDA  = 18;
constexpr uint8_t I2C_SCL  = 19;
constexpr uint8_t I2C_WIRE = 0;  // Default Wire (SDA = 18, SCL = 19)

// LEDs
constexpr uint8_t LED1 = 2;
constexpr uint8_t LED2 = 3;
constexpr uint8_t LED3 = 4;

// GPS
constexpr uint8_t GPS_TX        = 7;
constexpr uint8_t GPS_RX        = 8;
constexpr uint8_t GPS_EN        = 9;
constexpr uint8_t GPS_HW_SERIAL = 2;  // Using TX2 and RX2

// Adafruit RFM95 Radio
constexpr uint8_t RADIO_CS  = 0;
constexpr uint8_t RADIO_RST = 28;
constexpr uint8_t RADIO_EN  = 29;
constexpr uint8_t RADIO_G0  = 17;
constexpr uint8_t RADIO_G1  = 16;
constexpr uint8_t RADIO_G2  = 15;
constexpr uint8_t RADIO_G3  = 14;
constexpr uint8_t RADIO_G4  = 41;
constexpr uint8_t RADIO_G5  = 40;
constexpr uint8_t RADIO_INT = RADIO_G0; // Interrupt pin
constexpr uint8_t RADIO_SPI = HW_SPI1;

// RYLR998 UART radio
constexpr uint8_t RADIO2_TX        = 34;
constexpr uint8_t RADIO2_RX        = 35;
constexpr uint8_t RADIO2_RST       = 36;
constexpr uint8_t RYLR998_HW_SERIAL = 8;  // Using TX8 and RX8
