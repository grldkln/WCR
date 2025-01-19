#include <Arduino.h>
#include "gloabls.h"
#include "defines.h"

double inputPWM;

void setMotorSpeedD1(double speed)
{
    speed = abs(speed);
    if(speed < 0)
    {
        analogWrite(PIN_DRIVER_1A, speed)
        analogWrite(PIN_DRIVER_1B, 0)
    }
    if(speed > 0)
    {
        analogWrite(PIN_DRIVER_1A, 0)
        analogWrite(PIN_DRIVER_1B, speed)
    }
}

void setMotorSpeedD2(double speed)
{
    speed = abs(speed);
    if(speed < 0)
    {
        analogWrite(PIN_DRIVER_2A, speed)
        analogWrite(PIN_DRIVER_2B, 0)
    }
    if(speed > 0)
    {
        analogWrite(PIN_DRIVER_2A, 0)
        analogWrite(PIN_DRIVER_2B, speed)
    }
}