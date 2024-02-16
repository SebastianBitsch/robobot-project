/***************************************************************************
*   Copyright (C) 2017 by DTU                             *
*   jca@elektro.dtu.dk                                                    *
*
*   Main function for small regulation control object (regbot)
*   build on Teensy,
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

#define REV_ID "$Id: main.cpp 943 2018-05-08 20:05:27Z jcan $"

// #include "IntervalTimer.h"
#include "core_pins.h"
#include "usb_serial.h"

/// main loop counter
uint32_t mainLoop = 0;

#define PIN_LED_DEBUG 13

// ////////////////////////////////////////

void initialization()   // INITIALIZATION
{
  pinMode ( PIN_LED_DEBUG, OUTPUT );
  // init USB connection (parameter is not used - always 12MB/s)
  Serial.begin ( 115200 ); // USB init serial
  analogWriteResolution ( 12 ); // set PWM resolution
  // init AD converter
  // init CPU cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
extern "C" int main ( void )
{
  // run initialization
  initialization();
  int n = 0, v = 0;
  const int MSL = 50;
  char s[MSL];
  uint32_t usedTime = 0, thisCycleStartTime = 0;
  Serial.write("Hello\n");
  while ( true ) 
  { // main loop
    n++;
    snprintf(s, MSL, "%d, %d, %gms\n", n, v, float(usedTime) * 1000.0 / float(F_CPU) );
    Serial.write(s);
    v = analogRead(9);
    delay(1);
    if (n % 2000 == 0)
      digitalWriteFast(PIN_LED_DEBUG, 1);
    if (n % 2000 == 1000)
      digitalWriteFast(PIN_LED_DEBUG, 0);
    usedTime = ARM_DWT_CYCCNT - thisCycleStartTime;
    thisCycleStartTime = ARM_DWT_CYCCNT;
  }
  return 0;
}

