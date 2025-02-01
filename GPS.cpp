#include "GPS.h"

GPS::GPS(uint8_t serial, uint32_t baud_rate) : gps(&HW_SERIAL(serial)) { gps.begin(baud_rate); }

GPS::~GPS() {}

void GPS::begin()
{
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Output RMC and GGA sentences
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Update every 1 second
    gps.sendCommand(PGCMD_ANTENNA);               // Request antenna status
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

std::pair<bool, GPSData&> GPS::read()
{
    if (gps.newNMEAreceived())
    {
        if (!gps.parse(gps.lastNMEA()))
        {
            return {false, data}; // Skip invalid data
        }
    }

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
