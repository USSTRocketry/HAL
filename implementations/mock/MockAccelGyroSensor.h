#pragma once

#include "abstractions/IAccelGyroSensor.h"
#include "types.h"
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

namespace HAL {

class AccelGyroSensor : public IAccelGyroSensor
{
private:
    AccelGyroData data;
    SensorStatus status;
    
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
    AccelGyroSensor(uint8_t /*i2c_addr*/ = 0x6A, uint8_t /*i2c_wire*/ = 0)
        : status(SensorStatus::Success), data{20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f}
#if defined(USST_PLATFORM_MOCK)
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_accelgyro.csv")
#endif
    {
    }

    AccelGyroSensor(uint8_t /*cs*/, uint8_t /*miso*/, uint8_t /*mosi*/, uint8_t /*sck*/)
        : status(SensorStatus::Success), data{20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f}
#if defined(USST_PLATFORM_MOCK)
        , currentIndex_(0), useFlightData_(false), dataFilePath_("data/flight_accelgyro.csv")
#endif
    {
    }

    virtual ~AccelGyroSensor() = default;

    SensorStatus begin() override
    {
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

    const AccelGyroData& read() override
    {
        if (useFlightData_) {
            if (currentIndex_ < flightData_.size()) {
                data = flightData_[currentIndex_].sensorData;
                currentIndex_++;
                return data;
            } else {
                if (!flightData_.empty()) {
                    data = flightData_.back().sensorData;
                    currentIndex_ = flightData_.size();
                }
                useFlightData_ = false;
            }
        }
        return data;
    }

    void setData(const AccelGyroData& Data)
    {
        data = Data;
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

} // namespace HAL
