/***************************************************************************
*   Copyright (C) 2019-2022 by DTU                             *
*   jca@elektro.dtu.dk                                                    *
*
*   Main function for small regulation control object (regbot)
*   build on Teensy 3.1 or higher,
*   intended for 31300/1 Linear control
* 
* The MIT License (MIT)  https://mit-license.org/
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
* is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies 
* or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
* THE SOFTWARE. */

// #define REV_ID "$Id: regbot.cpp 1625 2023-11-25 18:43:22Z jcan $"


#define MISSING_SYSCALL_NAMES

#include <malloc.h>
#include "pins.h"
#include "IntervalTimer.h"
#include "ulog.h"
#include "src/main.h"
#include "umission.h"
#include "ulinesensor.h"
#include "ueeconfig.h"
#include "wifi8266.h"
#include "uservo.h"
#include "usound.h"

#include "uusb.h"
#include "ustate.h"
#include "uencoder.h"
#include "umotor.h"
#include "umotortest.h"
#include "uad.h"
#include "ucurrent.h"
#include "uirdist.h"
#include "uimu2.h"
#include "udisplay.h"
#include "uusbhost.h"


#ifdef REGBOT_HW4
#pragma message "COMPILED REGBOT for version 4 PCB (Teensy 3.5) "
#elif defined(REGBOT_HW41)
#pragma message "COMPILED REGBOT for Teensy4.1 on PCB version 5 or 6.3"
#elif defined(REGBOT_HW63_35)
#pragma message "COMPILED REGBOT for Teensy3.5 on PCB version 6.3 (no linesensor support)"
#else
#pragma message "COMPILED REGBOT v3 and older (Teensy 3.2)"
#endif


// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period - typically 1ms)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
volatile uint32_t tsec = 0; /// time that will not overrun
volatile uint32_t tusec = 0; /// time that will stay within [0...999999]
// flag for start of new control period
volatile bool startNewCycle = false;
int pushHBlast = 0;
float steerWheelAngle = 0;
// Heart beat interrupt service routine
void hbIsr ( void );
///


// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  ad.setup();
  state.setup();
  state.setStatusLed(HIGH);
//   sound.setup();
// #ifdef REGBOT_HW41
//   sound.play(2); // jingle 2 (HW 7-8 only)
//   delay (1500); // ms (for the welcome tune)
// #endif
  usb.setup();
  command.setup();
  encoder.setup();
  ls.setup();
  irdist.setup();
  userMission.setup();
  control.setup();
  imu2.setup();
  usbhost.setup();
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr, ( unsigned int ) 10 ); // heartbeat timer, value in usec
  // data logger init
  logger.setup();
  logger.setLogFlagDefault();
  logger.initLogStructure ( 100000 / state.CONTROL_PERIOD_10us );
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  eeConfig.setup();
  // configuration changes setup
  current.setup();
  servo.setup();  // set PWM for available servo pins
  motor.setup();  // set motor pins
  motortest.setup();
  // display on green-board (regbot 5.0 (HW 7-8)) requires reboot after setting HW version
  display.setup();
  // start heartbeat to USB
  state.decode("sub hbt 400\n");
  state.setStatusLed(LOW);
}


// int debugPos = 0;
// uint32_t debugTime = 0;
int debugSaved = 0;
/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
void loop ( void )
{
  control.resetControl();
  bool cycleStarted = false;
  state.setStatusLed(LOW);
  int debugSaved2 = 0;
  int debugCnt = 0;
//   sound.play(2);
  // then just:
  // - listen for incoming commands
  //   and at regular intervals (1ms)
  // - read sensors,
  // - run control
  // - implement on actuators
  // - do data logging
  while ( true ) 
  { // main loop
    usb.tick(); // service commands from USB
//     debugTime = hb10us;
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle ) // start of new control cycle
    { // error detect
      if (debugSaved > 0)
      {
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# timing late, detected at position  %d\n", debugSaved);
        usb.send(s);
        debugSaved2 = debugSaved;
        debugSaved = 0;
        debugCnt++;
      }
      if (control.debugMissionEnd and debugCnt > 0)
      { // ending the mission
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# timing late, detected at position %d (count=%d)\n", debugSaved2, debugCnt);
        debugSaved2 = 0;
        usb.send(s);
        control.debugMissionEnd = false;
        debugCnt = 0;
      }
      startNewCycle = false;
      cycleStarted = true;
      // AD converter should start as soon as possible, to also get a reading at half time
      // values are not assumed to change faster than this
      ad.tick();
      state.timing(1);
      encoder.tick();
      current.tick();
      imu2.tick();
      // record read sensor time
      state.timing(2);
      // calculate sensor-related values
      ls.tick();
      irdist.tick();
      // advance mission
      userMission.tick();
      // do control
      control.tick();
      // Implement on actuators
      servo.tick();
      motor.tick();
      motortest.tick();
      state.tick();
      command.tick();
      // record read sensor time + control time
      state.timing(3);
      // non-critical functions
      logger.tick();
//       sound.tick();
      display.tick();
      usbhost.tick();
    }
    // loop end time
    if (cycleStarted)
    { // control tick do not service subscriptions, so now is the time
//       control.subscribeTick();
      state.timing(4);
      state.saveCycleTime();
      cycleStarted = false;
    }
  }
//   return 0;
}

/**
* Heartbeat interrupt routine
* schedules data collection and control loop timing.
* */
void hbIsr ( void ) // called every 10 microsecond
{ // as basis for all timing
  hb10us++;
  tusec += 10;
  if (tusec > 1000000)
  {
    tsec++;
    tusec = 0;
  }
//   if (hb10us - debugTime > 150)
//   {
//     debugSaved = debugPos;
//   }
  if ( hb10us % state.CONTROL_PERIOD_10us == 0 ) // main control period start
  {
    userMission.missionTime += 1e-5 * state.CONTROL_PERIOD_10us;
    hbTimerCnt++;
    startNewCycle = true;
    state.timing(0);
  }
  if ( int(hb10us % state.CONTROL_PERIOD_10us) == state.CONTROL_PERIOD_10us/2 ) // start half-time ad conversion
  {
    ad.tickHalfTime();
  }
}

/////////////////////////////////////////////////////////////////
