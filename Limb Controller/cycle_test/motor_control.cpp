#include <Arduino.h>
//include "gloabls.h"
#include "defines.h"


void setMotorSpeedD1(double speed)
{
    if(speed <= -1)
    {
        analogWrite(PIN_DRIVER_1A, abs(speed));
        analogWrite(PIN_DRIVER_1B, 0);
    }
    if(speed >= 1)
    {
        analogWrite(PIN_DRIVER_1A, 0);
        analogWrite(PIN_DRIVER_1B, speed);
    }
    if(speed == 0)
    {
      analogWrite(PIN_DRIVER_1A, 0);
      analogWrite(PIN_DRIVER_1B, 0);
    }
}

void setMotorSpeedD2(double speed)
{
    if(speed <= -1)
    {
        analogWrite(PIN_DRIVER_2A, abs(speed));
        analogWrite(PIN_DRIVER_2B, 0);
    }
    if(speed >= 1)
    {
        analogWrite(PIN_DRIVER_2A, 0);
        analogWrite(PIN_DRIVER_2B, speed);
    }
    if(speed == 0)
    {
      analogWrite(PIN_DRIVER_2A, 0);
      analogWrite(PIN_DRIVER_2B, 0);
    }
}