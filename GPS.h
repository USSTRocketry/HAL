#pragma once

#include <Adafruit_GPS.h>
#include <HardwareSerial.h>

#include "types.h"


class GPS
{

private:
    Adafruit_GPS gps;
public:
    GPSData data;

public:
    GPS(HardwareSerial* serial, uint32_t baud_rate = 9600);
    ~GPS();

    void begin();
    void configure(uint32_t update_rate_ms = 1000, const char* output_mode = PMTK_SET_NMEA_OUTPUT_RMCONLY);
    GPSData* read();
    bool hasFix();
    void sendCommand(const char* command);
};
