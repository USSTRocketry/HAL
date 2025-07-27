#pragma once
#include <list>


enum ErrorCodes{
    NOERR = 0,
    INIT_BMP_ERR = 1,
	INIT_ACCELGYRO_ERR = 2,
	INIT_MAGNETOMETER_ERR = 3,
	INIT_THERMOMETER_ERR = 4,
	INIT_GPS_ERR = 5,
	INIT_RADIO_ERR = 6,
	INIT_SD_ERR = 7
};


class DebugLights
{
private:
    /* data */
    ErrorCodes CurrentErrorCode = NOERR;
    std::list<ErrorCodes> Errors;
    int timeCounter = 0;
    int SwitchTimeMsecs = 1000;

    void WriteErrorCodeToLEDs(int ErrorCode);

    public:

    DebugLights(/* args */);
    ~DebugLights();

    void AddError(ErrorCodes err);
    void ClearErrors();
    void UpdateLights();
    bool HasErrors();
};




