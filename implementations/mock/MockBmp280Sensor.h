#pragma once

#include "abstractions/IBmp280Sensor.h"
#include "types.h"
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

namespace HAL {

class Bmp280Sensor : public IBmp280Sensor
{
private:
    BMP280Data data;
    SensorStatus status;
    
    struct DataPoint {
        float timestamp;
        float temperature;
        float pressure;
        float altitude;
    };
    
    std::vector<DataPoint> flightData_;
    size_t currentIndex_;
    bool useFlightData_;
    std::string dataFilePath_;

public:
    // Match Arduino constructors (I2C and SPI)
    Bmp280Sensor(uint8_t i2c_addr = 0x77, uint8_t i2c_wire = 0, float /*sea_level_hpa*/ = 0.0f)
        : status(SensorStatus::Success), data{20.5f, 101325.0f, 100.0f}
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_bmp280.csv")
    {
        static_cast<void>(i2c_addr);
        static_cast<void>(i2c_wire);
    }

    Bmp280Sensor(uint8_t /*cs*/, uint8_t /*miso*/, uint8_t /*mosi*/, uint8_t /*sck*/, float /*sea_level_hpa*/ = 0.0f)
        : status(SensorStatus::Failure), data{20.5f, 101325.0f, 100.0f}
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_bmp280.csv")
    {
    }

    virtual ~Bmp280Sensor() = default;

    SensorStatus begin() override
    {
        // Autoload default flight data for simulation
        if (flightData_.empty()) {
            const bool loaded = loadFromFile(dataFilePath_.c_str());
            status = loaded ? SensorStatus::Success : SensorStatus::Failure;
        }
        return status;
    }

    SensorStatus getStatus() const override
    {
        return status;
    }

    const BMP280Data& read() override
    {
        if (currentIndex_ < flightData_.size()) {
            const auto& point = flightData_[currentIndex_];
            data.temperature = point.temperature;
            data.pressure = point.pressure;
            data.altitude = point.altitude;
            currentIndex_++;
            return data;
        } else {
            // Hold last sample once data is exhausted
            if (!flightData_.empty()) {
                const auto& point = flightData_.back();
                data.temperature = point.temperature;
                data.pressure = point.pressure;
                data.altitude = point.altitude;
                currentIndex_ = flightData_.size();
            }
        }
        return data;
    }

    void setStatus(SensorStatus newStatus)
    {
        status = newStatus;
    }
    
    bool loadFromFile(const char* filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return false;
        }
        
        flightData_.clear();
        std::string line;
        
        std::getline(file, line);
        if (line.find("timestamp") == std::string::npos) {
            file.seekg(0);
        }
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            DataPoint point;
            char comma;
            
            if (ss >> point.timestamp >> comma >> 
                      point.temperature >> comma >> 
                      point.pressure >> comma >> 
                      point.altitude) {
                flightData_.push_back(point);
            }
        }
        
        currentIndex_ = 0;
        useFlightData_ = !flightData_.empty();
        dataFilePath_ = filepath;
        
        return useFlightData_;
    }
    
    void resetPlayback()
    {
        currentIndex_ = 0;
    }
};

} // namespace HAL
