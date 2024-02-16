/***************************************************************************
*   Copyright (C) 2019 by DTU                             *
*   jca@elektro.dtu.dk                                                    *
*
*   Main function for small regulation control object (regbot)
*   build on Teensy 3.1 or higher,
*   intended for 31300/1 Linear control
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

// #define REV_ID "$Id: main.cpp 1029 2019-03-10 12:27:18Z jcan $"

#include <malloc.h>

#include "main.h"

// #include "../teensy3/kinetis.h"
// #include "../teensy3/pins_arduino.h"
// #include "../teensy3/core_pins.h"
#include "pins.h"
#include "mpu9150.h"
#include "motor_controller.h"
#include "data_logger.h"
#include "control.h"
#include "robot.h"
#include "mission.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "wifi8266.h"
#include "command.h"
#include "servo.h"
#include "subscribe.h"
#include "tunes.h"
#include <sys/time.h>
//#include <../Snooze/Snooze.h>
//#define __MK20DX256__

#pragma message "REGBOT for WEBOTS only"
// dummy functions to allow minimal edit for webots
void digitalWriteFast(int pin, bool a) {};
bool digitalReadFast(int pin) {return true; };
void analogWrite(int pin, int val) {};
void pinMode(int pin, int mode) {};
int  analogRead(int pin) { return 111;};
int   digitalRead(int pin) { return 0;};  

int16_t robotId = 0;
uint8_t robotHWversion = 6;
timeval t1, t2, ts1, ts2, dt;
float maxSampleTime = 0, minSampleTime = 1000.0;
float sumSampleTime = 0;
int countSampleTime = 0;

// main heartbeat timer to service source data and control loop interval
double SAMPLETIME = 0.001; // sample time in seconds (0.00001 * CONTROL_PERIOD_10us);
/// has positive reply been received frrom IMU
bool imuAvailable = false;
// battery low filter
uint16_t batVoltInt = 0;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (1ms) control_period
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
// flag for start of new control period
volatile bool startNewCycle = false;
uint32_t startCycleCPU;
//
uint32_t controlUsedTime[3] = { 0, 0, 0 }; // third item is acceleration limit is reached (active)
int pushHBlast = 0;
bool batteryHalt = false;
float steerWheelAngle = 0;
// Heart beat interrupt service routine
uint16_t adcInt0Cnt = 0;
uint16_t adcInt1Cnt = 0;
uint16_t adcStartCnt = 0, adcHalfCnt = 0;
// Destination for the first 5 ADC conversions. Value is set in ADC interrupt routine
uint16_t * adcDest[ADC_NUM_NO_LS] =
{
  &irRawAD[0],
  &irRawAD[1],
  &batVoltInt,
  &motorCurrentM[0],
  &motorCurrentM[1]

};
// List of AD numbers. First 5 values are ID, Battery and motor current. Remaining are the 8 line-sensor values
int adcPin[ADC_NUM_ALL] =
{
  PIN_IR_RAW_1,
  PIN_IR_RAW_2,
  PIN_BATTERY_VOLTAGE,
  PIN_LEFT_MOTOR_CURRENT,
  PIN_RIGHT_MOTOR_CURRENT,
  PIN_LINE_SENSOR_0,
  PIN_LINE_SENSOR_1,
  PIN_LINE_SENSOR_2,
  PIN_LINE_SENSOR_3,
  PIN_LINE_SENSOR_4,
  PIN_LINE_SENSOR_5,
  PIN_LINE_SENSOR_6,
  PIN_LINE_SENSOR_7
};
int adcSeq = 0;
bool adcHalf; // for double ADC conversion for LS
uint32_t adcStartTime, adcConvertTime;
uint32_t adcHalfStartTime, adcHalfConvertTime;

/// all control implementations (tick etc)
UControl control;
// the main mission class
UMission userMission;

UServo servo;

USubscribe subscribe;
/// main loop counter
uint32_t mainLoop = 0;

#ifdef REGBOT_HW4
  const int EEPROM_SIZE = 4048;
#else
  const int EEPROM_SIZE = 2024;
#endif


// ////////////////////////////////////////

void initialization()   // INITIALIZATION
{
  // sample time is determined by simulator (units of ms)
  SAMPLETIME = robot->getBasicTimeStep() / 1000.0;
  motorInit();           // init ports PWM out, direction out, encoder in
  servo.initServo();     // set PWM for available servo pins
  lineSensorOn = false;
  // data logger init
  setLogFlagDefault();
  initLogStructure ();
  // init encoder interrupts
  imuAvailable = true; //MPU9150_init();
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  eeConfig.fileConfigLoad(false);
  //
  motorCurrentM[0] = 0;
  motorCurrentM[1] = 0;
  batVoltInt = 11.1;
  motorCurrentMLowPass[0] = 0;
  motorCurrentMLowPass[1] = 0;  
}

/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
  uint8_t m = 0;
  int ltc, lastTimerCnt = 0; /// heartbeat timer loop count
  uint32_t loggerRowWait = 0;
  int row = -1;
  uint32_t thisCycleStartTime = 0;
  uint32_t debug1adcIntCntLast = 0, adcIntErrCnt = 0;
  int adcResetCnt = 0;
  bool cycleStarted = false;
  // power on

void mainInit()
{ // WEBOT init functions
  batteryUse = false;
  // run initialization
  initialization();
  //n = 0;
  motorAnkerVoltage[0] = 0.0;
  motorAnkerVoltage[1] = 0.0;
  // addMotorVoltage(0, 0);
  motorSetAnchorVoltage();
  control.resetControl();

  // time starts now (in seconds)
  missionTime = 0.0;
  // then just:
  // - listen for incoming commands
  // then every 1 ms (new cycle):
  // - read sensors,
  // - run control
  // - implement on actuators
  // - do data logging
  initSocketServer();
  // end of init
}


// Loop is called from regbot_3.cpp
// every time step (probably 4 ms)
bool loopTick()
{
  double time = robot->getTime();
  hbTimerCnt = int(time * 1000);
  float dtf;
  if (mainLoop < 5)
    printf("# loopTick(): %f sec, hb=%d\n", time, hbTimerCnt);
  
  startNewCycle = true;
    // get data from usb//wifi - if available
    handleIncoming();
    //
    ltc = hbTimerCnt;
    if ( startNewCycle ) 
    { // start of new control cycle
      startNewCycle = false;
      cycleStarted = true;
      // read new sensor values
      readSensors();
      // battery flat check
      if ( batteryUse ) // keep an eye on battery voltage
      {
        batteryMonitoring();
      }
      // record read sensor time
      //
      // calculate sensor-related values
      updatePose(mainLoop);
      estimateTilt();
      estimateIrDistance();
      estimteLineEdgePosition();
      if ( not encTimeScaleCalibrated and encTimeTryCalibrate and ( ( hbTimerCnt % 21 ) == 0 ) )
        calibrateEncoderTest();
      //
      // mission time control
      if ( control.missionState == 0 ) // we are waiting to start, so time since start is zero
      {
        timeAtMissionStart = hb10us;
        if ( missionTime > 18000.0 )
          // restart time (after 5 hours) - max usable time is 32768, as added in 1ms bits
          missionTime = 0.0;
      }
      // do control
      control.controlTick();
      // Implement on actuators
      servo.servoTick();
      if (control.controlActive )
      {
        steerWheelAngle = servo.setServoSteering();
        motorSetAnchorVoltage();
      }
      // send subscribed messages
      subscribe.sendToSubscriber();

      gettimeofday(&t2, NULL);
      timersub(&t2, &t1, &dt);
      dtf = dt.tv_sec * 1000.0 + dt.tv_usec/1000.0;
      if (dtf > maxSampleTime)
        maxSampleTime = dtf;
      if (dtf < minSampleTime)
        minSampleTime = dtf;
      sumSampleTime += dtf;
      countSampleTime++;
      t1 = t2;
      
      // log data at requested interval
      if ((ltc - lastTimerCnt ) >= logInterval or control.chirpRun)
      {
        bool doLog = not control.chirpRun;
        if (not doLog)
        { // we log anyhow, if we are dooing chirp modulation
          if (control.chirpLog)
          { // time to do a log action
            control.chirpLog = false;
            doLog = true;
          }
        }
        // timing in ms
        dataloggerExtra[0] = SAMPLETIME*1000.0;
        dataloggerExtra[1] = float(ltc - lastTimerCnt);
        dataloggerExtra[2] = sumSampleTime/float(countSampleTime);
        dataloggerExtra[3] = minSampleTime;
        dataloggerExtra[4] = maxSampleTime;
        maxSampleTime = 0;
        minSampleTime = 1000.0;
        countSampleTime = 0;
        sumSampleTime = 0.0;
        if (doLog)
        {
          lastTimerCnt = ltc;
          m++;
          if (loggerLogging())
          {
            setStatusLed ( ( m & 0xff ) < 128 );
            stateToLog();
          }
        }
      }
    }
    // send logged data to client
    if (logToUSB) // send log to USB or wifi
    {
      printf("Writing log to client - a total of %d rows (row=%d)\n", logRowCnt, row);
      //if ((hbTimerCnt - loggerRowWait ) > 10 ) // attempt to wait a bit after a few lines send to logger
      {
        // but do not seem to work, so set to just 10ms wait after 8 rows
        int row20 = 0;
        // signal log read using on-board LED
        setStatusLed ( HIGH );
        // transfer 8 rows at a time
        if (true)
        {
          for (row20 = 0; row < logRowCnt; row20++ ) // write buffer log to destination
          {
            row = logWriteBufferTo (row);
            row++;
            if ( not logToUSB )
              break;
//             printf("Row %d of %d\n", row, logRowCnt);
          }
        }
        // set pause time
        loggerRowWait = hbTimerCnt;
        if ( row >= logRowCnt ) // finished
        {
          logToUSB = false;
          row = -1;
        }
        setStatusLed ( LOW );
        printf("Writing log to client finished - a total of %d rows\n", logRowCnt);
      }
    }
    mainLoop++;
    if ( loggerLogging() )
      setStatusLed ( ( ltc & 0xff ) < 127 );
    else
      setStatusLed ( ( ltc & 0x3ff ) < 50 );
    // loop end time
    if (cycleStarted)
    {
      cycleStarted = false;
    }
  return 0;
}

