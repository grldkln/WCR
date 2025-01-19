#include "defines.h"
#include "motor_control.h"
#include "globals.h"

#include <TCA9548.h>
#include <AS5600.h>

#define IN219

#ifdef IN226
#include <INA226.h>
INA226 pump_monitor(0x40);
#endif

#ifdef IN219
#include <Adafruit_INA219.h>
Adafruit_INA219 pump_monitor(0x40);
#endif

#include <Wire.h>
#include <PID_v2.h>

#include "string.h"



#define PRINT_VAL
#define CONNECT_INA //comment to bypass

PCA9546 multiPlex(0x70);
AS5600 middle_encoder;
AS5600 frame_encoder;
PID_v2 middleMotorControl(Kp, Ki, Kd, PID::Direct);
PID_v2 frameMotorControl(Kp, Ki, Kd, PID::Direct);

double mValue;
double fValue;
int pValue;
int prevValue;


void setup() 
{
  pinMode(PIN_RELAY     , OUTPUT        );
  pinMode(PIN_DRIVER_1A , OUTPUT        );
  pinMode(PIN_DRIVER_1B , OUTPUT        );
  pinMode(PIN_DRIVER_2A , OUTPUT        );
  pinMode(PIN_DRIVER_2B , OUTPUT        );
  pinMode(PIN_LS        , INPUT         );

  Serial.begin(9600);
  // Serial.println("=========== Debugging Begin ===========");
  // Serial.println(" Searching Multiplexer Channels");
  // Serial.print( multiPlex.channelCount());
  // Serial.print(" Channels Found." );
  digitalWrite(PIN_RELAY, RELAY_OFF);

  //I2C Init
  //Serial.println("Initializing I2C Network");
  //Serial.flush();
  //WRCT = 0;
  Wire.flush();
  Wire.end();
  delay(1000);
  Wire.setClock(10000);
  Wire.begin();  // put your setup code here, to run once:

  if (!pump_monitor.begin() )
  {
    //Serial.println("Could not connect to INA226.");
  }

  #ifdef IN226
  pump_monitor.setMaxCurrentShunt(8, 0.01);
  pump_monitor.setAverage(INA226_16_SAMPLES);
#endif

  digitalWrite(PIN_RELAY, RELAY_ON);
}

void loop() 
{
    //Serial.print("Pump Current :");
    Serial.println(pump_monitor.getCurrent_mA(), 3);
    delay(50);

}
