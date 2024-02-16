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

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H

#define REGBOT

#include <string.h>
#include <stdlib.h>
#include <usb_serial.h>
#include <core_pins.h>
#include <HardwareSerial.h>
#include <ADC.h>

// control period is time between control calls
// and is in units of 10us, ie 125 is 1.25ms or 800Hz
#define CONTROL_PERIOD_10us 100
#define SAMPLETIME  (0.00001 * CONTROL_PERIOD_10us)

extern int robotId; // robot number [1..999]
extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt; /// in ms (not assumed to overflow)
extern bool motorPreEnabled;
extern int usbWriteFullCnt;
// ADC setup
const bool useADCInterrupt = true;
const int useADCresolution = 12;
const float useADCMaxADCValue = 4096*2;
// AD conversion
const int MAX_ADC = 13; // used ADC channels
extern ADC * adc;       // ADC class
extern int adcSeq;      // current ADC measurement index - shifted in interrupt-routine
extern bool adcHalf;    // for double ADC conversion for LS
extern int adcPin[MAX_ADC]; // pin sequence for ADC MUX
extern uint16_t adcStartCnt, adcHalfCnt; // time count for conversion start
extern uint32_t adcStartTime, adcConvertTime;
extern uint32_t adcHalfStartTime, adcHalfConvertTime;
// Is communication with IMU possible (if not a power cycle is needed)
extern bool imuAvailable;
/// battery halt is when battery voltage is too low ,
/// mission is stopped and 12V power is cut off.
/// if on USB power, then the processor continues.
extern bool batteryHalt;

#endif