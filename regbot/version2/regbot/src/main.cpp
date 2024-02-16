/***************************************************************************
 *   Copyright (C) 2014 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
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
 
#define REV_MAIN 2
#define REV "$Rev: 359 $"
#define REV_MINOR 3

#define REV_ID "$Id: main.cpp 359 2016-07-17 18:58:36Z jcan $" 

#include <malloc.h>
#include <ADC.h>
#include "IntervalTimer.h"
#include "mpu9150.h"
#include "motor_controller.h"
#include "data_logger.h"
#include "control.h"
#include "robot.h"
#include "main.h"
#include "mission.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "wifi8266.h"
#include "command.h"
//#include <../Snooze/Snooze.h>
//#define __MK20DX256__

int robotId = 0;
/** main heartbeat timer to service source data and control loop interval */
IntervalTimer hbTimer;
/// has positive reply been received frrom IMU
bool imuAvailable = false;
// battery low filter
int batLowCnt = 0;
uint16_t batVoltInt = 0;
// flag reset at when motor is enabled first time
bool motorPreEnabled = 1;
//
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
// flag for start of new control period
volatile bool startNewCycle = false;
uint16_t controlUsedTime[2] = {0,0};
int pushHBlast = 0;
bool batteryHalt = false;
/**
 * Heart beat interrupt service routine */
void hbIsr(void);
///
ADC * adc = new ADC();
uint16_t adcInt0Cnt = 0;
uint16_t adcInt1Cnt = 0;
uint16_t adcStartCnt = 0, adcHalfCnt = 0;
const int LS_FIRST = 5;
uint16_t * adcDest[LS_FIRST] = {&irRaw[0], &irRaw[1], &batVoltInt, &motorCurrentM[0], &motorCurrentM[1]};
int adcPin[MAX_ADC] = {1, 0, 9, 10, 11, 12, 13, 26, 27, 28, 29, 30, 31};
int adcSeq = 0; 
bool adcHalf; // for double ADC conversion for LS
uint32_t adcStartTime, adcConvertTime;
uint32_t adcHalfStartTime, adcHalfConvertTime;

// ////////////////////////////////////////

void initialization()
{ // INITIALIZATION
  pinMode(LED_BUILTIN, OUTPUT);
  // init USB connection (parameter is not used - always 12MB/s
  Serial.begin(115200); 
  // init serial to ardulog
  Serial1.begin(115200);    // connection to wifi serial connection (should be fast and no error)
  // init motor control
  motorInit();           // init ports PWM out, direction out, encoder in
  // init AD converter
  if (useADCInterrupt)
  { // AD using interrupt
    adc->adc0->setReference(ADC_REF_1V2);
    adc->adc1->setReference(ADC_REF_1V2);
    // not needed
    adc->adc0->recalibrate();
    adc->adc1->recalibrate();
    //
    adc->setResolution(12, 0);
    adc->setResolution(12, 1);
    adc->setConversionSpeed(ADC_MED_SPEED, 0);
    adc->setConversionSpeed(ADC_MED_SPEED, 1);
  }
  else
  { // AD poll
    analogReference(INTERNAL);
  }
  // more pins
  pinMode(6,INPUT); // start switch - version 2B
  pinMode(23,INPUT); // battery voltage
  pinMode(11,INPUT); // start switch - version 2A
  pinMode(18,OUTPUT); // line sensor LED full power
  pinMode(32,OUTPUT); // line sensor LED half power - or power to IR (with new power board)
  pinMode(33,OUTPUT); // all power off if low
  pinMode18 = OUTPUT;
  //
  pinMode(A12,INPUT); // Line sensor sensor value
  pinMode(A13,INPUT); // Line sensor sensor value
  pinMode(A15,INPUT); // Line sensor sensor value
  pinMode(A16,INPUT); // Line sensor sensor value
  pinMode(A17,INPUT); // Line sensor sensor value
  pinMode(A18,INPUT); // Line sensor sensor value
  pinMode(A19,INPUT); // Line sensor sensor value
  pinMode(A20,INPUT); // Line sensor sensor value
  lineSensorOn = false;
  digitalWriteFast(18, lineSensorOn);
  // power on
  digitalWriteFast(33, true);
  
  // start 10us timer (heartbeat timer)
  hbTimer.begin(hbIsr, (unsigned int)  10); // heartbeat timer, value in usec
  //
  // data logger init
  setLogFlagDefault();
  initLogStructure(100000 / CONTROL_PERIOD_10us);
  // init encoder interrupts
  attachInterrupt(M1ENC_A, m1EncoderA, CHANGE);
  attachInterrupt(M2ENC_A, m2EncoderA, CHANGE);
  attachInterrupt(M1ENC_B, m1EncoderB, CHANGE);
  attachInterrupt(M2ENC_B, m2EncoderB, CHANGE);
  //
  // I2C init
  // Setup for Master mode, pins 18/19, external pullups, 400kHz
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  // Initialization of MPU9150
  digitalWriteFast(LED_BUILTIN,1);  
  imuAvailable = MPU9150_init();
  digitalWriteFast(LED_BUILTIN,0);
  if (not imuAvailable)
    usb_send_str("# failed to find IPU\r\n");  
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  if (true)
    eeConfig.eePromLoadStatus(NULL);
  //
  if (imuAvailable)
  {
    if (true)
    {
      bool isOK = mpuRequestData();
      if (not isOK)
        usb_send_str("#first ACC read request failed\r\n");
      isOK = mpuReadData();
      if (not isOK)
        usb_send_str("#first ACC read failed (too)\r\n");
    }
    else
    {
      int a = readAccGyro();
      if (a != 0)
        usb_send_str("#first ACCGyro failed\r\n");
    }
    readMagnetometer();
  }
  if (useADCInterrupt)
  { // initialize analogue values
    motorCurrentM[0] = adc->analogRead(10);
    motorCurrentM[1] = adc->analogRead(11);
    batVoltInt = adc->analogRead(23);
    // initialize low-pass filter for current offset
    motorCurrentMLowPass[0] = adc->analogRead(10) * 100;
    motorCurrentMLowPass[1] = adc->analogRead(11) * 100;
    // enable interrupt for the remaining ADC oprations
    adc->enableInterrupts(0);
    adc->enableInterrupts(1);
  }
  else
  {
    motorCurrentM[0] = analogRead(10);
    motorCurrentM[1] = analogRead(11);
    // battery voltage
    batVoltInt = analogRead(23);
    // initialize low-pass filter for current offset
    motorCurrentMLowPass[0] = analogRead(10) * 100;
    motorCurrentMLowPass[1] = analogRead(11) * 100;
  }
  
}

/**
 * Main loop
 * primarily for initialization,
 * non-real time services and
 * synchronisation with heartbeat.*/
extern "C" int main(void)
{
  uint8_t m = 0;
  uint32_t loop = 0;
  int ltc, lastTimerCnt = 0; /// heartbeat timer loopcoun
  uint32_t loggerRowWait = 0;
  int row = 0;
  uint32_t start10us;
  // debug main
//   if (false)
//     debugLoop();
  // debug end
  // wait a bit - to allow usb to connect in order to see init errors
  delay(1000); // ms
  // 
  initialization();
  //n = 0;
  motorAnkerVoltage[0] = 0.0;
  motorAnkerVoltage[1] = 0.0;
  // addMotorVoltage(0, 0);
  motorSetAnchorVoltage();
  // turn on linesensor diodes
  digitalWriteFast(18, LOW);
  // turn on IR sensor
  // NB this should be on during a mission only, or
  // when the GUI needs it for calibration
  digitalWriteFast(32, useDistSensor);
  // time starts now (in seconds)
  time = 0.0;
  //
  //usb_send_str("#1\n");
  // then just listen for commands and process them
  while (1)
  { // main loop
    // get data from usb//wifi - if available
    handleIncoming();
    //
    if (wifi.setup < 0 and hbTimerCnt > 3000)
    { // wait 3 seconds before setup of wifi
      wifi.setup = 1;
    }
    if (wifi.setup > 0 and wifi.setup < 99)
      wifi.serialSetup();
    //
    ltc = hbTimerCnt;
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle)
    { // start of new control cycle
      startNewCycle = false;
      start10us = hb10us;      
      // read new sensor values
      readSensors();
      // battery flat check
      if (batteryUse)
      { // keep an eye on battery voltage - if on USB, then battery is between 0 and 3 Volts - no error
        if (batteryOff)
        { // battery may be back on
          if (batVoltInt >= (batteryIdleVoltageInt) or not batteryHalt)
          { // battery is high or switch on command
            if (not batteryHalt)
              batLowCnt = 0;
            else
              batLowCnt--;
            if (batLowCnt <= 0)
            { // stop processor to save a bit more current
              usb_send_str("# Power back on\r\n");
              // turn power on if new powerboard is installed
              digitalWriteFast(33, true);
              batteryOff = false;
              // batteryHalt = false;
              // stop processor - goes down to about 30mA@12V (from about 60mA@12V) with buck-boost converter
              // stopTeensy();
            }
          }
        }
        else if ((batVoltInt < batteryIdleVoltageInt and batVoltInt > int(4.5 / batVoltIntToFloat)) or batteryHalt)
        {
          batLowCnt++;
          if (batLowCnt % 1000 == 100 and batLowCnt < 10000 )
          { // send warning first 10 seconds and stop mission
            usb_send_str("# Battery low - going POWER OFF in 10 second!\r\n");
            missionStop = true;
          }
          if (batLowCnt > 10000 or batteryHalt)
          { // stop processor to save a bit more current
            usb_send_str("# Battery low! (shut down all but USB power!)\r\n");
            // turn power off if new powerboard is installed
            digitalWriteFast(33, false);
            batteryOff = true;
            batLowCnt = 3000;
            batteryHalt = true;
            // stop processor - goes down to about 30mA@12V (from about 60mA@12V) with buck-boost converter
            // stopTeensy();
          }
        }
        else
          batLowCnt = 0;
      }
      // record read sensor time
      controlUsedTime[0] = hb10us - start10us;
      // calculate sensor-related values
      updatePose(loop);
      estimateTilt();
      estimateIrDistance();
      if (lineSensorOn)
        findLineEdge();
      else
      {
        lsLeftValid = false;
        lsRightValid = false;
        crossingBlackLine = false;
        crossingWhiteLine = false;
      }
      //
      if (missionState == 0)
      {  // we are waiting to start, so time since start is zero
        timeAtMissionStart = hb10us;
        if (time > 18000.0)
          // restart time (after 5 hours) - max usable time is 32768, as added in 1ms bits
          time = 0.0;
        // and allow manual setting of anchor voltage
      }
      // do control
      controlTick();
      // implement the new controlled motor voltage
      motorSetAnchorVoltage();
      //
      // record read sensor time + control time
      controlUsedTime[1] = hb10us - start10us;
    }
    //
    if ((pushInterval > 0) && (ltc - pushTimeLast) >= pushInterval)
    { // send to USB and first active wifi client
      requestingClient = -2;
      pushTimeLast = ltc;
      // pack and send one status message
      sendStatus();
    }
    // send hart beat anyhow - if not in interactive mode from putty
    else if (localEcho < 1 and ((ltc - pushHBlast) > 667))
    { // send hart beat every second - if no other communication
      requestingClient = -2;
      pushHBlast = ltc;
      sendHartBeat();
    }
    if ((ltc - lastTimerCnt) >= logInterval)
    {
      lastTimerCnt = ltc;
      m++;
      if (loggerLogging())
      {
        digitalWriteFast(LED_BUILTIN,(m & 0xff) < 128);
        stateToLog();
      }
      else if (logToUSB and (hbTimerCnt - loggerRowWait) > 10)
      { // attempt to wait a bit after a few lines send to logger
        // but do not seem to work, so set to just 10ms wait after 8 rows
        int row20 = 0;
        // signal log read using on-board LED
        digitalWriteFast(LED_BUILTIN,HIGH);
        // transfer 8 rows at a time
        for (row20 = 0; row < logRowCnt /*and row20 < 8*/; row20++)
        { // write buffer log to destination
          row = logWriteBufferTo(row);
          row++;
          if (not logToUSB)
            break;
        }
        // set pause time
        loggerRowWait = hbTimerCnt;
        if (row >= logRowCnt)
        { // finished
          logToUSB = false;
          row = 0;
        }
        digitalWriteFast(LED_BUILTIN,LOW);
      }
    }
    loop++;
    if (loggerLogging())
      digitalWriteFast(LED_BUILTIN,(ltc & 0xff) < 127);
    else
      digitalWriteFast(LED_BUILTIN,(ltc & 0x3ff) < 10);
  }
  return 0;
}

/**
 * Heartbeat interrupt routine
 * schedules data collection and control loop timing.
 * */
void hbIsr(void)
{ // called every 10 microsecond
  // as basis for all timing
  hb10us++;
  if (hb10us % CONTROL_PERIOD_10us  == 0)
  { // 1ms timing - main control period start
    time += 1e-5 * CONTROL_PERIOD_10us; 
    hbTimerCnt++;
    startNewCycle = true;
    //
    // overload for encoder speed based on period
    if (not encTimeOverload[0])
      if ((int32_t)hb10us - (int32_t)encStartTime[0] > 256*8*256)
        encTimeOverload[0] = true;
    if (not encTimeOverload[1])
      if ((int32_t)hb10us - (int32_t)encStartTime[1] > 256*8*256)
        encTimeOverload[1] = true;
  } 
  if (hb10us % CONTROL_PERIOD_10us  == 60)
  { // start half-time ad conversion
    if (adcSeq >= MAX_ADC)
    {
      adcHalfStartTime = hb10us;
      adcSeq = 0;
      /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[0], -1); // + 300;
      adcHalf = true;
      adcHalfCnt++;
    }
  }
}

/////////////////////////////////////////////////////////////////

// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr() 
{
  uint16_t v = 0;
  v = adc->adc0->readSingle();
  if (adcSeq < LS_FIRST)
    // low-pass filter non-line sensor values at about 2ms time constant
    // result is in range 0..8196 (for measured between 0v and 1.2V)
    *adcDest[adcSeq] = ((*adcDest[adcSeq]) >> 1) + v;
  else if (adcHalf)
    // line sensor raw value
    adcLSH[adcSeq - LS_FIRST] = v;
  else
    adcLSL[adcSeq - LS_FIRST] = v;
  adcSeq++;
  if (adcSeq < MAX_ADC)
  { // start new and re-enable interrupt
    /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[adcSeq], -1); // + 100;
  }
  else
  { // finished
    if (adcHalf)
    {
      adcHalfConvertTime = hb10us - adcHalfStartTime;
      digitalWriteFast(18, lineSensorOn);
      digitalWriteFast(32, lineSensorOn);
    }
    else
    {
      adcConvertTime = hb10us - adcStartTime;
      digitalWriteFast(18, LOW);
      digitalWriteFast(32, LOW);
    }
  }
  adcInt0Cnt++;
}

//////////////////////////////////////////////////////////

void adc1_isr() 
{
  uint16_t v = adc->adc1->readSingle();
  if (adcSeq < LS_FIRST)
    *adcDest[adcSeq] = ((*adcDest[adcSeq]) >> 1) + v;
  else if (adcHalf)
    adcLSH[adcSeq - LS_FIRST] = v;
  else
    adcLSL[adcSeq - LS_FIRST] = v;
  adcSeq++;
  if (adcSeq < MAX_ADC)
  { // start new and re-enable interrupt
    /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[adcSeq], -1); // + 200;
  }
  else
  { // finished
    if (adcHalf)
    {
      adcHalfConvertTime = hb10us - adcHalfStartTime;
      digitalWriteFast(18, lineSensorOn);
      digitalWriteFast(32, lineSensorOn);
    }
    else
    {
      adcConvertTime = hb10us - adcStartTime;
      digitalWriteFast(18, LOW);
      digitalWriteFast(32, LOW);
    }
  }
  adcInt1Cnt++;
}


