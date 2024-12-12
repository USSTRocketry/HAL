#pragma once

#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <cstdint>

#include "types.h"
#include "PINS.h"


class GPS
{

private:
    Adafruit_GPS gps;
public:
    GPSData data;

public:
    GPS(uint8_t serial = GPS_HW_SERIAL, uint32_t baud_rate = 9600);
    ~GPS();

    void begin();
    void configure(uint32_t update_rate_ms = 1000, const char* output_mode = PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPSData* read();
    bool hasFix();
    void sendCommand(const char* command);
};
