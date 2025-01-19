#ifndef DEFINES_H_
#define DEFINES_H_

//I2C Channel
#define AS5600_MIDDLE 0
#define AS5600_FRAME 1

//Pin Numbers
#define PIN_RELAY 7
#define PIN_DRIVER_1A 5
#define PIN_DRIVER_1B 6
#define PIN_DRIVER_2A 10
#define PIN_DRIVER_2B 11
#define PIN_LS 4

#define RELAY_ON LOW
#define RELAY_OFF HIGH

#define SENSOR226


#define ROT_THRES 1.0 // degrees

#define MAX_MESSAGE_LENGTH 12 // char

#ifdef SENSOR226
#define ADHESION_POINT 6650 //mA
#endif

#ifdef SENSOR219
#define ADHESION_POINT -1350//mA
#endif



#endif