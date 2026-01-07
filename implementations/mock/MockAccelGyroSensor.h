#pragma once

#include "abstractions/IAccelGyroSensor.h"
#include "types.h"
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

class MockAccelGyroSensor : public IAccelGyroSensor
{
private:
    AccelGyroData data;
    uint8_t status;
    
    struct DataPoint {
        float timestamp;
        AccelGyroData sensorData;
    };
    
    std::vector<DataPoint> flightData_;
    size_t currentIndex_;
    bool useFlightData_;
    std::string dataFilePath_;

public:
    // Match Arduino constructors (I2C and SPI)
    MockAccelGyroSensor(uint8_t /*i2c_addr*/ = 0x6A, uint8_t /*i2c_wire*/ = 0)
        : status(1), data{20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f}
#if defined(USST_PLATFORM_MOCK)
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_accelgyro.csv")
#endif
    {
    }

    MockAccelGyroSensor(uint8_t /*cs*/, uint8_t /*miso*/, uint8_t /*mosi*/, uint8_t /*sck*/)
        : status(1), data{20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f}
#if defined(USST_PLATFORM_MOCK)
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_accelgyro.csv")
#endif
    {
    }

    virtual ~MockAccelGyroSensor() = default;

    uint8_t begin() override
    {
        if (flightData_.empty()) {
            loadFromFile(dataFilePath_.c_str());
        }
        return status;
    }

    uint8_t getStatus() const override
    {
        return status;
    }

    AccelGyroData* read() override
    {
        if (useFlightData_) {
            if (currentIndex_ < flightData_.size()) {
                data = flightData_[currentIndex_].sensorData;
                currentIndex_++;
                return &data;
            } else {
                if (!flightData_.empty()) {
                    data = flightData_.back().sensorData;
                    currentIndex_ = flightData_.size();
                }
                useFlightData_ = false;
            }
        }
        return &data;
    }

    void setMockData(const AccelGyroData& mockData)
    {
        data = mockData;
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
                      point.sensorData.temperature >> comma >>
                      point.sensorData.gyroX >> comma >>
                      point.sensorData.gyroY >> comma >>
                      point.sensorData.gyroZ >> comma >>
                      point.sensorData.accelX >> comma >>
                      point.sensorData.accelY >> comma >>
                      point.sensorData.accelZ) {
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
    
    size_t getDataPointCount() const
    {
        return flightData_.size();
    }
    
    bool isUsingFlightData() const
    {
        return useFlightData_;
    }
};
