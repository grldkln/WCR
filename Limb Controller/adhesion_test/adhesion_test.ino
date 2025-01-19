/*

Pump testing code only used to test the adhesion of the robot
attached to a test surface (whiteboard) to see if the structure
is strong enough or if the adhesion holds.

*/ 

#include "defines.h"
#include <string.h>

#include <TCA9548.h>
#include <AS5600.h>
#include <INA226.h>
#include <Wire.h>

PCA9546 multiPlex(0x70);
AS5600 middle_encoder;
AS5600 frame_encoder;
INA226 pump_monitor(0x40);

void setup() 
{
  pinMode(PIN_RELAY     , OUTPUT        );
  pinMode(PIN_DRIVER_1A , OUTPUT        );
  pinMode(PIN_DRIVER_1B , OUTPUT        );
  pinMode(PIN_DRIVER_2A , OUTPUT        );
  pinMode(PIN_DRIVER_2B , OUTPUT        );
  pinMode(PIN_LS        , INPUT         );

  Serial.begin(9600);
  Serial.println("To turn pump on and off please write [ON|OFF] .");
}

void loop() 
{
  while(Serial.available() == 0)
  {
    String input = String(Serial.readString());
    if( input == "ON" )
    {
      digitalWrite(PIN_RELAY, RELAY_ON);
      Serial.println("Pump On");
    }

    if( input == "OFF")
    {
      digitalWrite(PIN_RELAY, RELAY_OFF);
      Serial.println("Pump Off");
    }
    else
    {
      Serial.println("Unknown Command");
    }
  }
}
