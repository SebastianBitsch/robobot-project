/***************************************************************************
 *   Copyright (C) 2015 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Line sensor functions
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

#include "main.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "pins.h"
/** sensor 0 (called 1 in interface) is normally looking to the left (pin 1(gnd),2(+), and 3(analog value) in plug)
 *  sensor 1 (called 2 in interface) is normally looking forward (pin 4,5,6 in plug) */
uint16_t irRaw[2]; 
bool useDistSensor = false;
bool distSensorInstalled = true;
//
float irDistance[2];
uint16_t irCal20cm[2] = {3000, 3000}; 
uint16_t irCal80cm[2] = {850, 850};

float irA[2] = {-0.04, -0.04};
float irB[2] = {711.0, 711.0};

void sendStatusDistIR()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL, "irc %.3f %.3f %d %d %d %d %d %d %d %d\r\n" ,
           irDistance[0], irDistance[1],
           irRaw[0], irRaw[1],
           irCal20cm[0], irCal80cm[0],
           irCal20cm[1], irCal80cm[1],
           useDistSensor,
           distSensorInstalled
  );
  usb_send_str(reply);
}

bool setIrCalibrate(const char * buf)
{
  bool used = false; 
  { // is for the line sensor
    if (strncmp(buf, "irc", 3) == 0)
    { // assumed white line
      char * p1 = (char *)&buf[4];
      used = true;
      irCal20cm[0] = strtol(p1, &p1, 10);
      irCal80cm[0] = strtol(p1, &p1, 10);
      irCal20cm[1] = strtol(p1, &p1, 10);
      irCal80cm[1] = strtol(p1, &p1, 10);
      setIRpower(strtol(p1, &p1, 10));
      distSensorInstalled = strtol(p1, &p1, 10);
      //usb_send_str("# got a lip\n");
      for (int i = 0; i < 2; i++)
      {
        irA[i] = 0.2 * (irCal20cm[i] - 4 * irCal80cm[i]) / (irCal20cm[i] - irCal80cm[i]);
        irB[i] = 0.6 * irCal20cm[i] * irCal80cm[i] / (irCal20cm[i] - irCal80cm[i]);
      }
    }
  }
  return used;  
}


void estimateIrDistance()
{
  if (useDistSensor)
  { // dist sensor has power, so estimate
    if (irRaw[0] > 0)
      irDistance[0] = irA[0] + irB[0]/irRaw[0];
    if (irRaw[1] > 0)
      irDistance[1] = irA[1] + irB[1]/irRaw[1];
  }
  else
  { // not installed or not on (set to far away 10m)
    irDistance[0] = 10.0;
    irDistance[1] = 10.0;
  }
}

/////////////////////////////////////

void eePromSaveIr()
{
  uint8_t f = 0;
  if (useDistSensor) f = 1 << 0;
  if (distSensorInstalled) f += 1 << 1;
  eeConfig.pushByte(f);
  eeConfig.pushWord(irCal20cm[0]);
  eeConfig.pushWord(irCal20cm[1]);
  eeConfig.pushWord(irCal80cm[0]);
  eeConfig.pushWord(irCal80cm[1]);
}

/////////////////////////////////////

void eePromLoadIr()
{
  int f = eeConfig.readByte();
  distSensorInstalled = f & (1 << 1);
  setIRpower(f & (1 << 0));
  int skipCount = 2 + 2 + 2 + 2;
  //
  if (not eeConfig.isStringConfig())
  { // load from flash
    irCal20cm[0] = eeConfig.readWord();
    irCal20cm[1] = eeConfig.readWord();
    irCal80cm[0] = eeConfig.readWord();
    irCal80cm[1] = eeConfig.readWord();
  }
  else
    // load from hard-coded mission
    eeConfig.skipAddr(skipCount);
}

void setIRpower(bool power)
{
  if (distSensorInstalled)
    useDistSensor = power;
  else
    useDistSensor = false;
  //
  digitalWriteFast(PIN_POWER_IR, useDistSensor);
  // debug
//   const int MSL=50;
//   char s[MSL];
//   snprintf(s, MSL, "# set ir power (32) to %d\r\n", power);
//   usb_send_str(s);
  // debug end
}