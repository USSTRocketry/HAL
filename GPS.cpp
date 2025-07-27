#include "GPS.h"

<<<<<<< HEAD
GPS::GPS(uint8_t serial, uint32_t baud_rate) : gps(&HW_SERIAL(serial)) { gps.begin(baud_rate); }
=======
GPS::GPS(uint8_t serial, uint32_t baud_rate)
: gps(&HW_SERIAL(serial))
, baud_rate(baud_rate)
{ 
}
>>>>>>> main

GPS::~GPS() {}

bool GPS::begin()
{
<<<<<<< HEAD
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Output RMC and GGA sentences
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Update every 1 second
    gps.sendCommand(PGCMD_ANTENNA);               // Request antenna status
=======
    if (gps.begin(baud_rate)){
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // Output RMC and GGA sentences
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Update every 1 second
        gps.sendCommand(PGCMD_ANTENNA);                // Request antenna status
        return true;
    }else{
        return false;
    }
>>>>>>> main
}

// Configure GPS settings
void GPS::configure(uint32_t update_rate_ms, const char* output_mode)
{
    // Set NMEA output format (e.g., RMC only, RMC+GGA, etc.)
    gps.sendCommand(output_mode);

    // Set GPS update rate
    switch (update_rate_ms)
    {
        case 1000:
            gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
            break;
        case 500:
            gps.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
            break;
        case 200:
            gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
            break;
        case 100:
            gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
            break;
        default:
            gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
            break;
    }
}

<<<<<<< HEAD
std::pair<bool, GPSData&> GPS::read()
{
    if (gps.newNMEAreceived())
    {
        if (!gps.parse(gps.lastNMEA()))
        {
            return {false, data}; // Skip invalid data
=======
void GPS::update()
{
    // Update GPS data
    char c = gps.read();
    if (gps.newNMEAreceived()) {
        if (!gps.parse(gps.lastNMEA())) {
            return;  // Skip invalid data
>>>>>>> main
        }
    }
}

GPSData* GPS::read()
{
    // Read GPS data
    // char c = gps.read();
    // if (c) Serial.printf("GPS CHAR: %c\n", c);
    // if (gps.newNMEAreceived()) {
    //     if (!gps.parse(gps.lastNMEA())) {
    //         return nullptr;  // Skip invalid data
    //     }
    // }

    // Serial.print("\nTime: ");
    // if (gps.hour < 10) { Serial.print('0'); }
    // Serial.print(gps.hour, DEC); Serial.print(':');
    // if (gps.minute < 10) { Serial.print('0'); }
    // Serial.print(gps.minute, DEC); Serial.print(':');
    // if (gps.seconds < 10) { Serial.print('0'); }
    // Serial.print(gps.seconds, DEC); Serial.print('.');
    // if (gps.milliseconds < 10) {
    //   Serial.print("00");
    // } else if (gps.milliseconds > 9 && gps.milliseconds < 100) {
    //   Serial.print("0");
    // }
    // Serial.println(gps.milliseconds);
    // Serial.print("Date: ");
    // Serial.print(gps.day, DEC); Serial.print('/');
    // Serial.print(gps.month, DEC); Serial.print("/20");
    // Serial.println(gps.year, DEC);
    // Serial.print("Fix: "); Serial.print((int)gps.fix);
    // Serial.print(" quality: "); Serial.println((int)gps.fixquality);

    if (hasFix())
    {
        data = {.latitude    = gps.latitudeDegrees,
                .longitude   = gps.longitudeDegrees,
                .altitude    = gps.altitude, // Altitude in meters
                .speed       = gps.speed,    // Speed in knots
                .angle       = gps.angle,    // Course in degrees
                .satellites  = gps.satellites,
                .fix_quality = gps.fixquality};

        return {true, data};
    }

    return {false, data};
}

// Check if GPS has a fix
bool GPS::hasFix() { return gps.fix; }

// Send custom command to GPS
void GPS::sendCommand(const char* command) { gps.sendCommand(command); }
