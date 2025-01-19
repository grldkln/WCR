#include "defines.h"
#include "motor_control.h"
#include "globals.h"

#include <TCA9548.h>
#include <AS5600.h>

#ifdef SENSOR226
#include <INA226.h>
INA226 pump_monitor(0x40);
#endif

#ifdef SENSOR219
#include <Adafruit_INA219.h>
Adafruit_INA219 pump_monitor(0x40);
#endif

#include <Wire.h>
#include <PID_v2.h>

#include "string.h"

#define RP4
//#define PLOT //uncomment and comment PRINT to allow serial plotting
#define PRINT //comment to disable printing
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

  digitalWrite(PIN_RELAY, RELAY_OFF);
  Serial.begin(9600);

  //I2C Init
  #ifdef RP4
  Serial.println("Starting Controller");
  #endif
  #ifdef PRINT
  Serial.println("Initializing I2C Network");
  #endif
  Serial.flush();
  //WRCT = 0;
  Wire.flush();
  Wire.end();
  delay(1000);
  Wire.setClock(10000);
  Wire.begin();
  
  if(multiPlex.begin())
  {
    #ifdef PRINT
    Serial.println("Connected to TCA9546.");
    #endif
  }
  delay(200);
  
  multiPlex.selectChannel(AS5600_MIDDLE);
  if(middle_encoder.begin(4))
  {
    #ifdef PRINT
    Serial.println("Connected to AS5600 1.");
    #endif
  }
  //Wire.flush();
  delay(500);
  middle_encoder.setDirection(AS5600_CLOCK_WISE);

  multiPlex.selectChannel(AS5600_FRAME);
  if(frame_encoder.begin(4))
  {
    #ifdef PRINT
    Serial.println("Connected to AS5600 2.");
    #endif
  }
  //Wire.flush();
  delay(500);
  frame_encoder.setDirection(AS5600_CLOCK_WISE);


  if (!pump_monitor.begin() )
  {
    #ifdef PRINT
    Serial.println("Could not connect to INA226.");
    #endif
  }

  #ifdef SENSOR229
  pump_monitor.setMaxCurrentShunt(8, 0.01);
  pump_monitor.setAverage(INA226_16_SAMPLES);
  #endif

#ifdef PRINT
  Serial.println("I2C Devices Connected");
  #endif

// ZEROING SEQUANCE

  // Serial.println("Zeroing Modules");

  middleMotorControl.SetOutputLimits(-240, 240);
  middleMotorControl.SetSampleTime(10);
  // middleMotorControl.Start(middleEncoderAngle(), 0, 10);
  frameMotorControl.SetOutputLimits(-240, 240);
  frameMotorControl.SetSampleTime(10);
  // frameMotorControl.Start(frameEncoderAngle(), 0, 10);
  // while(middleEncoderAngle() >= 10)
  // {
  //   setMotorSpeedD1(middleMotorControl.Run(middleEncoderAngle()));
  // }
  // while(frameEncoderAngle() >= 10)
  // {
  //   setMotorSpeedD2(frameMotorControl.Run(frameEncoderAngle()));
  // }

  #ifdef PRINT
  Serial.println(" Current Encoder Level : ");
        Serial.println(middleEncoderAngle());
        Serial.println(frameEncoderAngle());

  Serial.println("Initialization Complete");
  #endif

  #ifdef RP4
  Serial.println("Init Complete");
  #endif

}

void loop() 
{
  Serial.flush();
  if (Serial.available() > 0) {
    // Read the incoming data
    char message = Serial.read();

    // Buffer to hold the command string
    char command[20];
    int index = 0;
    delay(500);
    // Read until a newline character is received
    while (message != '\n' && index <= sizeof(command) - 1) {
      command[index] = message;
      index++;
      if (Serial.available() > 0) {
        message = Serial.read();
      } else {
        break;
      }
    }
    // Null-terminate the command string
    command[index] = '\0';
    #ifdef PRINT
    Serial.println("New Command :");
    Serial.println(command);
    #endif
    #ifdef RP4
    Serial.println("Command Recieved");
    #endif

    char* secCom;
    char* thirdCom;

    mValue = strtod(command, &secCom);
    fValue = strtod(secCom, &thirdCom);
    pValue = strtod(thirdCom, NULL);
    

    if(pValue != prevValue)
    {
      switch(pValue)
      {
        case 0:
        digitalWrite(PIN_RELAY, RELAY_OFF);
        delay(3000);
        //while(pump_monitor.getCurrent_mA() >= ADHESION_POINT);
        break;
        case 1:
        digitalWrite(PIN_RELAY, RELAY_ON);
        //while(pump_monitor.getCurrent_mA() <= ADHESION_POINT);
        break;
      }
      prevValue = pValue;
    }
    Wire.flush();

      if (mValue >= 270 && mValue <= 340 && fValue >= 245 && fValue <= 310) 
      {
        #ifdef PRINT
        Serial.println(mValue); // Move motor M to the specified angle
        Serial.println(fValue); // Move motor F to the specified angle
        #endif

        bool middleRotate = true;
        bool frameRotate = true;
        middleMotorControl.Start(middleEncoderAngle(), 0, mValue);
        frameMotorControl.Start(frameEncoderAngle(), 0, fValue);
        delay(20);
        #ifdef PRINT
        Serial.println(" Current Encoder Level : ");
        Serial.println(middleEncoderAngle());
        Serial.println(frameEncoderAngle());
        #endif
        while(middleRotate || frameRotate )
        {
          if(middleEncoderAngle() <= (mValue - ROT_THRES) || middleEncoderAngle() >= (mValue + ROT_THRES))
          {
            setMotorSpeedD1(-(middleMotorControl.Run(middleEncoderAngle())));
          }
          else
          {
            middleRotate = false;
            setMotorSpeedD1(0);
          }
          if(frameEncoderAngle() <= (fValue - ROT_THRES) || frameEncoderAngle() >= (fValue + ROT_THRES))
          {
            setMotorSpeedD2(frameMotorControl.Run(frameEncoderAngle()));
          }
          else
          {
            frameRotate = false;
            setMotorSpeedD2(0);
          }
          #ifdef PLOT
            Serial.print(middleEncoderAngle());
            Serial.print(", ");
            Serial.print(mValue);
            Serial.print(", ");
            Serial.print(fValue);
            Serial.print(", ");
            Serial.println(frameEncoderAngle());
            #endif
        }
        #ifdef RP4
        Serial.println("Command Complete");
        #endif
      }
      else
      {
        #ifdef PRINT
        Serial.println("Error : Angle out of Bounds");
        #endif
      }
    } 
    setMotorSpeedD1(0);
    setMotorSpeedD2(0); 
}

double middleEncoderAngle()
{
  Wire.flush();
  multiPlex.selectChannel(AS5600_MIDDLE);
  return (middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES);
}

double frameEncoderAngle()
{
  Wire.flush();
  multiPlex.selectChannel(AS5600_FRAME);
  return (frame_encoder.readAngle() * AS5600_RAW_TO_DEGREES);
}