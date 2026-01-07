#pragma once

#include "abstractions/IBmp280Sensor.h"
#include "types.h"
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

class MockBmp280Sensor : public IBmp280Sensor
{
private:
    BMP280Data data;
    uint8_t status;
    
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
    MockBmp280Sensor(uint8_t i2c_addr = 0x77, uint8_t i2c_wire = 0, float /*sea_level_hpa*/ = 0.0f)
        : status(1), data{20.5f, 101325.0f, 100.0f}
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_bmp280.csv")
    {
    }

    MockBmp280Sensor(uint8_t /*cs*/, uint8_t /*miso*/, uint8_t /*mosi*/, uint8_t /*sck*/, float /*sea_level_hpa*/ = 0.0f)
        : status(0), data{20.5f, 101325.0f, 100.0f}
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_bmp280.csv")
    {
    }

    virtual ~MockBmp280Sensor() = default;

    uint8_t begin() override
    {
        // Autoload default flight data for simulation; ignore failures and keep defaults
        if (flightData_.empty()) {
            this->status = loadFromFile(dataFilePath_.c_str());
        }
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    BMP280Data* read() override
    {
        if (currentIndex_ < flightData_.size()) {
            const auto& point = flightData_[currentIndex_];
            data.temperature = point.temperature;
            data.pressure = point.pressure;
            data.altitude = point.altitude;
            currentIndex_++;
            return &data;
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
        return &data;
    }

    void setStatus(uint8_t newStatus)
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
