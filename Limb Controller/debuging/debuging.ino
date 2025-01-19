/* DEV NOTES

This file is for debugging the individual components such as
- 5V Relay
- INA226 + 5V Relay + Vacuum Pump
- BTS7960 + JGY370 + AS5600 + TCA9546

INA226 is calibrated with this variable
pump_monitor.setMaxCurrentShunt(8, 0.01);
having the best result, where the avg of 6000 meaxns good adhesion,
while under 5500 means no adheasion. Thi can be revised to be properlly
calibrated later on but will do for now.

AS5600 should be adjusted first using alignment procedures on the shaft.
The magnet for the AS5600 needs to be a diametrically polarized, not axially. This will affect the linearity of the sensor.

Carefull when wiring the L286N motor driver, there is a jumper which shorts the +12V to GND. (External Supply)
Measure current for the arm is 0.056 A or 56 mA on average.

The pumps average consumption is on avg 0.8A or 800mA on full adhesion to wall surface.

Known Issues:
- INA226 seems to have I2C issues. Consider checking the hardwiring.
- One prototype limb has a different processor (Atmega 168p), carefull when selecting boards.
- Arduino Nano 328p seems to use the oldbootloader, consider this when uploading.

*/
#include "defines.h"
#include "motor_control.h"

#include <TCA9548.h>
#include <AS5600.h>
#include <INA226.h>
#include <Wire.h>

//
#define DEBUG_RELAY
//#define DEBUG_INA226
//#define DEBUG_MOTOR
#define DEBUG_ENCODER


PCA9546 multiPlex(0x70);
AS5600 middle_encoder;
AS5600 frame_encoder;
INA226 pump_monitor(0x40);

void setup() {
  pinMode(PIN_RELAY     , OUTPUT        );
  pinMode(PIN_DRIVER_1A , OUTPUT        );
  pinMode(PIN_DRIVER_1B , OUTPUT        );
  pinMode(PIN_DRIVER_2A , OUTPUT        );
  pinMode(PIN_DRIVER_2B , OUTPUT        );
  pinMode(PIN_LS        , INPUT         );

  Serial.begin(9600);
  Serial.println("=========== Debugging Begin ===========");
  Serial.println(" Searching Multiplexer Channels");
  Serial.print( multiPlex.channelCount());
  Serial.print(" Channels Found." );
  digitalWrite(PIN_RELAY, RELAY_OFF);

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
  Serial.flush();
  multiPlex.selectChannel(AS5600_MIDDLE);
  if(middle_encoder.begin(4))
  {
    Serial.println("Connected to AS5600 1.");
  }
  Wire.flush();
  delay(500);
  middle_encoder.setDirection(AS5600_CLOCK_WISE);
  multiPlex.selectChannel(AS5600_FRAME);
  if(frame_encoder.begin(4))
  {
    Serial.println("Connected to AS5600 2.");
  }
  Wire.flush();
  delay(500);
  frame_encoder.setDirection(AS5600_CLOCK_WISE);
  if (pump_monitor.begin() )
  {
    Serial.println("Connected to INA226.");
  }
  delay(200);
  pump_monitor.setMaxCurrentShunt(8, 0.01);
  pump_monitor.setAverage(INA226_16_SAMPLES);
  Serial.println("I2C Devices Connected");

  Serial.println("Initialization Complete");

}

void loop() {
//============================ RELAY TEST
#ifdef DEBUG_RELAY
  Serial.println("Testing Relay.");
  delay(1000);
  digitalWrite(PIN_RELAY,RELAY_ON);
  delay(6000);
  digitalWrite(PIN_RELAY,RELAY_OFF);
  Serial.println("Relay Off");
  delay(1000);

  Serial.println("Relay Test Done");

#endif

//============================ INA226 TEST
#ifdef DEBUG_INA226
  Serial.println("Testing INA226.");
  Serial.println("\nBUS\tSHUNT\tCURRENT\tPOWER");
  digitalWrite(PIN_RELAY,RELAY_ON);
  for (int i = 0; i < 100; i++)
  {
    Serial.print(pump_monitor.getBusVoltage(), 3);
    Serial.print("\t");
    Serial.print(pump_monitor.getShuntVoltage_mV(), 3);
    Serial.print("\t");
    Serial.print(pump_monitor.getCurrent_mA(), 3);
    Serial.print("\t");
    Serial.print(pump_monitor.getPower_mW(), 3);
    Serial.println();
    delay(50);
  }
  digitalWrite(PIN_RELAY,RELAY_OFF);
  delay(5000);

#endif

//============================= MOTOR TEST

#ifdef DEBUG_MOTOR
//   Serial.println("Testing Motor");
//   Serial.println("Motor Forward");
//   setMotorSpeedD1(150);
//   setMotorSpeedD2(150);
//   delay(2000);
//   Serial.println("Motor Reverse");
//   setMotorSpeedD1(-150);
//   setMotorSpeedD2(-150);
//   delay(2000);
//   setMotorSpeedD1(0);
//   setMotorSpeedD2(0);
//   Serial.println("Motor Test Done");
//   delay(2000);

#endif

// //============================= ENCODER TEST
#ifdef DEBUG_ENCODER
  digitalWrite(PIN_RELAY, RELAY_ON);
  delay(2000);
  Serial.println("Testing Encoder");
  setMotorSpeedD1(-100);

  multiPlex.selectChannel(AS5600_MIDDLE);
  while((middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES) <= 340)
  {
  Serial.print("Middle Encoder Value : ");
  Serial.println(middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES);  
  }
  Serial.flush();
  setMotorSpeedD1(0);
  // multiPlex.selectChannel(AS5600_FRAME);
  // Serial.print("Frame Encoder Value : ");
  // Serial.println(frame_encoder.readAngle() *  AS5600_RAW_TO_DEGREES);
  // delay(10);
  
  delay(2000);
  setMotorSpeedD2(-100);
  multiPlex.selectChannel(AS5600_FRAME);
  while((frame_encoder.readAngle() * AS5600_RAW_TO_DEGREES) >= 245)
  {
  Serial.print("Frame Encoder Value : ");
  Serial.println(middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES);  
  }
  setMotorSpeedD2(0);
  Serial.flush();
  delay(2000);

  Serial.println("Switching Rotation");
  setMotorSpeedD1(100);
  delay(500);
  multiPlex.selectChannel(AS5600_MIDDLE);
  delay(100);
  while((middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES) >= 270)
  {
  Serial.print("Middle Encoder Value : ");
  Serial.println(middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES);  
  }
  setMotorSpeedD1(0);
  Serial.flush();

  delay(2000);
  setMotorSpeedD2(100);
  multiPlex.selectChannel(AS5600_FRAME);
  while((frame_encoder.readAngle() * AS5600_RAW_TO_DEGREES) <= 310)
  {
  Serial.print("Frame Encoder Value : ");
  Serial.println(middle_encoder.readAngle() * AS5600_RAW_TO_DEGREES);  
  }
  setMotorSpeedD2(0);
  Serial.flush();
  delay(2000);
  Serial.println("Encoder Test End");
  Wire.flush();
#endif
}

















