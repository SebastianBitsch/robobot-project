/***************************************************************************
 *   Copyright (C) 2019 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a teensy 3.1 72MHz ARM processor MK20DX256 - or any higher,  *
 *   intended for 31300/1 Linear control 1
 *   has an IMU and a dual motor controller with current feedback.
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H

//#define REV_MAIN 3
//#define REV "$Rev: 982 $" - moved to command.cpp (use getRevisionNumber())
/// Minor revision must be no bigger than 9 (not to overflow 16 bit integer when added to svn version number*10)
#define REV_MINOR 1

// debug
//#define REGBOT_HW4
#define WEBOT
// debug end

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/LED.hpp>
#include <webots/GPS.hpp>
// stubs and simulated hardware
// make 8, 16 and 16 bit integers compatible - GNU<->Teensy
typedef u_int8_t uint8_t;
typedef u_int16_t uint16_t;
typedef u_int32_t uint32_t;
// a few functions are needed here
#include <math.h>
#include <webots/Robot.hpp>
// support definitions
#define F_CPU 100000000
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
// connection to TEENSY main loop
// implemented in main.cpp
void mainInit();
int loop();
// stubs that are not available in simulation mode 
// but easier stub than to exclude using #define
// implemented in regbot_3.cpp
void digitalWriteFast(int pin, bool a);
bool digitalReadFast(int pin);
void analogWrite(int pin, int val);
void pinMode(int pin, int mode);
int  analogRead(int pin);
int   digitalRead(int pin);
// access to simulated robot structure
extern webots::Robot * robot;
const int WheelCnt = 2;
extern webots::Motor *wheels[WheelCnt];
extern webots::PositionSensor * wheelEncoder[WheelCnt];
extern webots::GPS * gps;
const int DistSensorCnt = 2;
extern webots::DistanceSensor *ps[DistSensorCnt];
const int LEDCnt = 2;
extern webots::LED * leds[LEDCnt];
extern webots::Accelerometer * accDev;
extern webots::Gyro * gyroDev;
const int LineSensorCnt = 8;
extern webots::DistanceSensor *lineSensorRaw[LineSensorCnt];


inline void setStatusLed(int value)
{
//   printf("# set LED to %d\n", value);
  if (value == 0)
    leds[0]->set(0);
  else
    leds[0]->set(1);
}

#include "pins.h"
// control period is time between control calls
// and is in units of 10us, ie 125 is 1.25ms or 800Hz
// #define CONTROL_PERIOD_10us 100

extern double SAMPLETIME; // in seconds

extern int16_t robotId; // robot number [1..999]
/** hw version 1 no line sensor nor wifi
 * hw 2 no power control on board, no wifi, 
 * hw 3 build in power control and wifi, and servo header - big motor controller
 * hw 4 same as 3, but small motor controller
 * hw 5 same as 2, but with sattelite power and wifi boards 
 * hw 6 is for version 4.0 (teensy 3.5) with integrated motor controller
 * */
extern u_int8_t robotHWversion; 

extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt; /// in ms (not assumed to overflow)
extern int usbWriteFullCnt;
// ADC setup
const bool useADCInterrupt = true;
const int useADCresolution = 12;
const float lpFilteredMaxADC = 4095*2;	// ADC returns 0->4095
// AD conversion
bool loopTick();
extern int adcSeq;      // current ADC measurement index - shifted in interrupt-routine
extern bool adcHalf;    // for double ADC conversion for LS
extern int adcPin[ADC_NUM_ALL]; // pin sequence for ADC MUX
extern uint16_t adcStartCnt, adcHalfCnt; // time count for conversion start
extern uint32_t adcStartTime, adcConvertTime;
extern uint32_t adcHalfStartTime, adcHalfConvertTime;
extern float steerWheelAngle; // steering angle (ref) from front wheel servo - calculated from wheel ref velocity
// Is communication with IMU possible (if not a power cycle is needed)
extern bool imuAvailable;
/// battery halt is when battery voltage is too low ,
/// mission is stopped and 12V power is cut off.
/// if on USB power, then the processor continues.
extern bool batteryHalt;
extern uint32_t mainLoop;
extern const int EEPROM_SIZE;


inline void setDebugLed(uint8_t value) {
}

class UControl;
extern UControl control;
// the main mission class
class UMission;
extern UMission userMission;

class UServo;
extern UServo servo;

class USubscribe;
extern USubscribe subscribe;

#endif
