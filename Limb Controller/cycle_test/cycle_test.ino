#include "defines.h"
#include "motor_control.h"
#include "globals.h"

#include <TCA9548.h>
#include <AS5600.h>
#include <INA226.h>
#include <Wire.h>
#include <PID_v2.h>

#include "string.h"



#define PRINT_VAL
#define CONNECT_INA //comment to bypass

PCA9546 multiPlex(0x70);
AS5600 middle_encoder;
AS5600 frame_encoder;
INA226 pump_monitor(0x40);
PID_v2 middleMotorControl(Kp, Ki, Kd, PID::Direct);
PID_v2 frameMotorControl(Kp, Ki, Kd, PID::Direct);

double mValue;
double fValue;
  bool middleRotate = true;
        bool frameRotate = true;



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
  Serial.println("Initializing I2C Network");
  //Serial.flush();
  //WRCT = 0;
  Wire.flush();
  Wire.end();
  delay(1000);
  Wire.setClock(10000);
  Wire.begin();
  
  if(multiPlex.begin())
  {
    Serial.println("Connected to TCA9546.");
  }
  delay(200);
  
  multiPlex.selectChannel(AS5600_MIDDLE);
  if(middle_encoder.begin(4))
  {
    Serial.println("Connected to AS5600 1.");
  }
  //Wire.flush();
  delay(500);
  middle_encoder.setDirection(AS5600_CLOCK_WISE);

  multiPlex.selectChannel(AS5600_FRAME);
  if(frame_encoder.begin(4))
  {
    Serial.println("Connected to AS5600 2.");
  }
  //Wire.flush();
  delay(500);
  frame_encoder.setDirection(AS5600_CLOCK_WISE);


  if (!pump_monitor.begin() )
  {
    Serial.println("Could not connect to INA226.");
  }
  #ifdef SENSOR229
  pump_monitor.setMaxCurrentShunt(8, 0.01);
  pump_monitor.setAverage(INA226_16_SAMPLES);
  #endif


  Serial.println("I2C Devices Connected");

// ZEROING SEQUANCE

  // Serial.println("Zeroing Modules");

  middleMotorControl.SetOutputLimits(-200, 200);
  middleMotorControl.SetSampleTime(10);
  // middleMotorControl.Start(middleEncoderAngle(), 0, 10);
  frameMotorControl.SetOutputLimits(-200, 200);
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
  Serial.println(" Current Encoder Level : ");
        Serial.println(middleEncoderAngle());
        Serial.println(frameEncoderAngle());

  Serial.println("Initialization Complete");

}

void loop() {
  middleRotate = true;
        frameRotate = true;
        mValue = 315;
        fValue = 308;
        middleMotorControl.Start(middleEncoderAngle(), 0, mValue);
        frameMotorControl.Start(frameEncoderAngle(), 0, fValue);
        digitalWrite(PIN_RELAY, RELAY_ON);
        delay(1000);
  while(middleRotate || frameRotate )
        {
          if(middleEncoderAngle() <= (mValue - ROT_THRES) || middleEncoderAngle() >= (mValue + ROT_THRES))
          {
            setMotorSpeedD1(-(middleMotorControl.Run(middleEncoderAngle())));
            //Serial.print(middleEncoderAngle());
            //Serial.print(",");
          }
          else
          {
            middleRotate = false;
            setMotorSpeedD1(0);
          }
          if(frameEncoderAngle() <= (fValue - ROT_THRES) || frameEncoderAngle() >= (fValue + ROT_THRES))
          {
            setMotorSpeedD2(frameMotorControl.Run(frameEncoderAngle()));
            //Serial.print(Frame Encoder:);
            //Serial.println(frameEncoderAngle());
          }
          else
          {
            frameRotate = false;
            setMotorSpeedD2(0);
          }
        }
        delay(3000);
        middleRotate = true;
        frameRotate = true;
        mValue = 335;
        fValue = 297;
        middleMotorControl.Start(middleEncoderAngle(), 0, mValue);
        frameMotorControl.Start(frameEncoderAngle(), 0, fValue);
  while(middleRotate || frameRotate )
        {
          if(middleEncoderAngle() <= (mValue - ROT_THRES) || middleEncoderAngle() >= (mValue + ROT_THRES))
          {
            setMotorSpeedD1(-(middleMotorControl.Run(middleEncoderAngle())));
            //Serial.print(middleEncoderAngle());
            //Serial.print(",");
          }
          else
          {
            middleRotate = false;
            setMotorSpeedD1(0);
          }
          if(frameEncoderAngle() <= (fValue - ROT_THRES) || frameEncoderAngle() >= (fValue + ROT_THRES))
          {
            setMotorSpeedD2(frameMotorControl.Run(frameEncoderAngle()));
            //Serial.print(Frame Encoder:);
            //Serial.println(frameEncoderAngle());
          }
          else
          {
            frameRotate = false;
            setMotorSpeedD2(0);
          }
        }
        middleRotate = true;
        frameRotate = true;
        digitalWrite(PIN_RELAY, RELAY_OFF);
        //while(pump_monitor.getCurrent_mA() <= ADHESION_POINT);
        delay(2000);
        mValue = 300;
        fValue = 290;
        middleMotorControl.Start(middleEncoderAngle(), 0, mValue);
        frameMotorControl.Start(frameEncoderAngle(), 0, fValue);
  while(middleRotate || frameRotate )
        {
          if(middleEncoderAngle() <= (mValue - ROT_THRES) || middleEncoderAngle() >= (mValue + ROT_THRES))
          {
            setMotorSpeedD1(-(middleMotorControl.Run(middleEncoderAngle())));
            //Serial.print(middleEncoderAngle());
            //Serial.print(",");
          }
          else
          {
            middleRotate = false;
            setMotorSpeedD1(0);
          }
          if(frameEncoderAngle() <= (fValue - ROT_THRES) || frameEncoderAngle() >= (fValue + ROT_THRES))
          {
            setMotorSpeedD2(frameMotorControl.Run(frameEncoderAngle()));
            //Serial.print(Frame Encoder:);
            //Serial.println(frameEncoderAngle());
          }
          else
          {
            frameRotate = false;
            setMotorSpeedD2(0);
          }
        }

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
