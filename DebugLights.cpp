#include "DebugLights.h"
#include <PINS.h>
#include <Arduino.h>

DebugLights::DebugLights(/* args */)
{
}

DebugLights::~DebugLights()
{
}

/// @brief adds an error to the list of errors
/// @param err the error that is added
void DebugLights::AddError(ErrorCodes err){
    Errors.push_back(err);
}

/// @brief Removes all stored error codes and resets the debug lights to off
void DebugLights::ClearErrors(){
    Errors.clear();
    CurrentErrorCode = NOERR;

    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,LOW);
}

// Returns true if any errors have been added.
bool DebugLights::HasErrors(){
    return Errors.size() > 0 || CurrentErrorCode != NOERR;
}

// Updates lights stste using an internal timer which swiches the state after 1 second to rotate through every error
void DebugLights::UpdateLights(){
    timeCounter += millis();

    if(timeCounter > SwitchTimeMsecs){
        timeCounter = 0;

        if(CurrentErrorCode != NOERR){
            Errors.push_back(CurrentErrorCode);
            CurrentErrorCode = Errors.front();
            Errors.pop_front();
        }
        else if (DebugLights::HasErrors()){
            CurrentErrorCode = Errors.front();
            Errors.pop_front();
        }

        WriteErrorCodeToLEDs(static_cast<int>(CurrentErrorCode));
    }
}

/// @brief Uses the first 3 bits of the inputed error code which is displayed on the three availible LEDs
/// @param ErrorCode The inputed error of which only the first 3 bits are used 
void DebugLights::WriteErrorCodeToLEDs(int ErrorCode){
    if((ErrorCode & 1) != 0){
        digitalWrite(LED1,HIGH);
    }else{
        digitalWrite(LED1,LOW);
    }

    if((ErrorCode & 2) != 0){
        digitalWrite(LED2,HIGH);
    }else{
        digitalWrite(LED2,LOW);
    }

    if((ErrorCode & 4) != 0){
        digitalWrite(LED3,HIGH);
    }else{
        digitalWrite(LED3,LOW);
    }
}