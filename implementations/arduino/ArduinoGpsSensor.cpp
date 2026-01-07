#include "implementations/arduino/ArduinoGpsSensor.h"
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>

// Macro for HardwareSerial mapping
#define HW_SERIAL(w) (w == 1 ? Serial1 \
                        : (w == 2 ? Serial2 \
                        : (w == 3 ? Serial3 \
                        : (w == 4 ? Serial4 \
                        : (w == 5 ? Serial5 \
                        : (w == 6 ? Serial6 \
                        : (w == 7 ? Serial7 \
                        : Serial8 \
                        )))))))

// Static instance of Adafruit_GPS
static Adafruit_GPS gps(&HW_SERIAL(1)); // Default to Serial1

ArduinoGpsSensor::ArduinoGpsSensor(uint8_t serial_port, uint32_t baud_rate)
    : serial_port(serial_port), baud_rate(baud_rate), status(0)
{
}

ArduinoGpsSensor::~ArduinoGpsSensor() {}

uint8_t ArduinoGpsSensor::begin()
{
    if (gps.begin(baud_rate)) {
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // Output RMC and GGA sentences
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Update every 1 second
        gps.sendCommand(PGCMD_ANTENNA);                // Request antenna status
        status = 1;
        return status;
    } else {
        status = 0;
        return status;
    }
}

uint8_t ArduinoGpsSensor::getStatus() const
{
    return status;
}

void ArduinoGpsSensor::configure(uint32_t update_rate_ms, const char* output_mode)
{
    // Set NMEA output format (e.g., RMC only, RMC+GGA, etc.)
    if (output_mode != nullptr) {
        gps.sendCommand(output_mode);
    }

    // Set GPS update rate
    switch (update_rate_ms) {
        case 1000: gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); break;
        case 500:  gps.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ); break;
        case 200:  gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); break;
        case 100:  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); break;
        default:   gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); break;
    }
}

void ArduinoGpsSensor::update()
{
    // Update GPS data
    char c = gps.read();
    if (gps.newNMEAreceived()) {
        if (!gps.parse(gps.lastNMEA())) {
            return;  // Skip invalid data
        }
    }
}

GPSData* ArduinoGpsSensor::read()
{
    if (hasFix()) {
        data.latitude = gps.latitudeDegrees;
        data.longitude = gps.longitudeDegrees;
        data.altitude = gps.altitude;  // Altitude in meters
        data.speed = gps.speed;       // Speed in knots
        data.angle = gps.angle;       // Course in degrees
        data.satellites = gps.satellites;
        data.fix_quality = gps.fixquality;
    } else {
        data.latitude = 0.0f;
        data.longitude = 0.0f;
        data.altitude = 0.0f;
        data.speed = 0.0f;
        data.angle = 0.0f;
        data.satellites = 0;
        data.fix_quality = 0;
    }

    return &data;
}

bool ArduinoGpsSensor::hasFix()
{
    return (gps.fix > 0) && (gps.fixquality > 0);
}

void ArduinoGpsSensor::sendCommand(const char* command)
{
    gps.sendCommand(command);
}
