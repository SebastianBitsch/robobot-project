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
 
#define REV_ID "$Id: main.cpp 402 2016-09-11 11:25:36Z jcan $" 

#include <malloc.h>
#include <ADC.h>
//#include "IntervalTimer.h"
#include <string.h>
#include <stdlib.h>
#include <usb_serial.h>
#include <core_pins.h>
#include <HardwareSerial.h>
//#define __MK20DX256__

#define hb_time_value 2
#define CONTROL_PERIOD_1ms 5000


int16_t robotId = 0;
uint8_t robotHWversion = 2;
int delay300nsCnt = 2;
volatile uint16_t delayActualCnt = 0;
/** main heartbeat timer to service source data and control loop interval */
IntervalTimer hbTimer;
/// has positive reply been received frrom IMU
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period)
volatile uint32_t hb03us = 0;     /// heart beat timer count (10 us)
uint32_t loop1frq, loop2frq;
// flag for start of new control period
volatile bool startNewCycle = false;
/**
 * Heart beat interrupt service routine */
void hbIsr(void);
///

bool missionStop = false;

const int LED_CNT = 12;
uint8_t ledValues[LED_CNT * 3];
//
int8_t localEcho = 0;
/**
 * usb command buffer space */
const int RX_BUF_SIZE = 200;
char usbRxBuf[RX_BUF_SIZE];
int usbRxBufCnt = 0;

bool newLedValue = false;

/** led pin on teensy to be used */
const uint8_t ledPin = 5;

/** 800 kHz LED cycle variabled */
typedef enum {LED_0, LED_03, LED_06, LED_09} LED_CYCLE;
bool led_cycle = false;
LED_CYCLE  ledCycleCnt = LED_0;
int ledIdx = 0;
uint8_t * led_value = ledValues;
int ledBit = 0;
bool ledBitVal = false;

void show();
// ////////////////////////////////////////

bool usb_send_str(const char * str) //, bool toUSB, bool toWifi)
{
  int n = strlen(str);
  bool okSend = true;
  // a human client, so send all to USB and other clients
  // surplus characters will be skipped
  usb_serial_write(str, n);
  return okSend;
}


void setLED(int led, uint8_t red, uint8_t green, uint8_t blue)
{
  if (led >= 0 and led < LED_CNT)
  {
    ledValues[led*3] = green;
    ledValues[led*3 + 1] = red;
    ledValues[led*3 + 2] = blue;
  }
  newLedValue = true;
}


void initialization()
{ // INITIALIZATION
  pinMode(LED_BUILTIN, OUTPUT);
  // init USB connection (parameter is not used - always 12MB/s
  Serial.begin(115200); 
  // init serial to ardulog
  Serial1.begin(115200);    // connection to wifi serial connection (should be fast and no error)
  //
  usb_send_str("# Ring LED controller\r\n");
//   { // AD poll
   analogReference(INTERNAL);
//   }
  // more pins
  pinMode(ledPin, OUTPUT); // LED data pin
  //
  // start 0.2us timer (heartbeat timer)
  usb_send_str("# starting HB interrupt\r\n");
//   hbTimer.begin(hbIsr, (uint32_t)  hb_time_value, true); // heartbeat timer, value in 0.1 usec units when fast=true
  //
  usb_send_str("# init clear leds\r\n");

  for (int i = 0; i < LED_CNT; i++)
    setLED(i, 10, 10, 10);

  usb_send_str("# init end\r\n");
  digitalWriteFast(LED_BUILTIN,0);
}


bool parse_and_execute_command(char * buf)
{
  const int MSL = 170;
  char s[MSL];
  char * p2;
  bool commandHandled = false;
  //
  if (strncmp(buf, "stop", 4) == 0)
  {
    missionStop = true;
    commandHandled = true;
  }
  else if (strncmp(buf, "wait ", 5) == 0)
  {
    int v = strtol(&buf[5], NULL, 0);
    commandHandled = true;
    snprintf(s, MSL, "# set busy wait from %d to %d\r\n", delay300nsCnt, v);
    usb_send_str(s);
    delay300nsCnt = v;
  }
  else if (strncmp(buf, "help", 4) == 0)
  {
    usb_send_str("# test to use LED ring with SK6812 LEDS\r\n");
    usb_send_str("# \r\n");
    snprintf(s, MSL, "running with HB interrupt F_BUS=%d value %d -> ITC=%d\r\n", F_BUS, hb_time_value, F_BUS / (10000000 / hb_time_value) - 1);
    usb_send_str(s);
    usb_send_str("# commands:\r\n");
    usb_send_str("#   help - this message\r\n");
    usb_send_str("#   set N R G B    Sets the Nth LED to these RGB values (0..255)\r\n");
    snprintf(s, MSL, "#   wait N         Sets busy-wait loop count to get accurate timing (is=%d)\r\n", delay300nsCnt);
    usb_send_str(s);
    usb_send_str("# (that is all)\r\n");
    commandHandled = true;
  }
  else if (strncmp(buf, "set ", 4) == 0)
  { // stop all 12V power (or turn on if 12 V power is off (and switch is on))
    p2 = &buf[4];
    int led = strtol(p2, &p2, 0);
    uint8_t red = strtol(p2, &p2, 0);
    uint8_t green = strtol(p2, &p2, 0);
    uint8_t blue = strtol(p2, &p2, 0);
    setLED(led, red, green, blue);
    // debug
    snprintf(s, MSL, "# setting led %d to r:%d, g:%d, b:%d\r\n", led, red, green, blue);
    usb_send_str(s);
    // debug end
    commandHandled = true;
  }
  else if (strncmp(buf, "get", 3) == 0)
  {
    for (int i = 0; i < LED_CNT; i++)
    {
      snprintf(s, MSL, "# led %2d r=%3d, g=%3d b=%3d\r\n", i, ledValues[i*3 + 1], ledValues[i*3], ledValues[i*3 + 2]);
      usb_send_str(s);
    }
  }
  else
  {
    snprintf(s, MSL, "# unhandled command '%s'\r\n", buf);
    usb_send_str(s);
  }
  return commandHandled;
}

/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void receivedCharFromUSB(uint8_t n)
{ // got another character from usb host (command)
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBuf[++usbRxBufCnt] = '\0';
  }
  if (localEcho == 1)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if (n == '\n' or n=='\r')
  { // zero terminate
    {
      if (usbRxBufCnt > 0)
      {
        usbRxBuf[usbRxBufCnt] = '\0';
        parse_and_execute_command(usbRxBuf);
      }
      if (localEcho == 1)
      {
        usb_send_str("\r\n>>");
        //usb_serial_flush_output();
      }
    }
    // flush remaining input
    usbRxBufCnt = 0;
  }
  else if (usbRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    usbRxBuf[usbRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    usb_send_str(msg);
    usbRxBufCnt = 0;
  }
}


void handleIncoming()
{
  int n;
  for (int i = 0; i < 10; i++)
  {
    n = usb_serial_getchar();
    if (n < 0)
      break;
    if (n >= '\n' and n < 0x80)
    { // command arriving from USB
      //usb_send_str("#got a char from USB\r\n");
      receivedCharFromUSB(n) ;
    }
  }
}

int ledNumber = 0;
int color = 0;
uint8_t ledi = 1;

void changeDisply()
{
  setLED(ledNumber, 0, 0, 0);
  ledNumber++;
  if (ledNumber >= LED_CNT)
  {
    ledNumber = 0;
    color ++;
  }
  switch (color)
  {
    case 0: setLED(ledNumber, ledi, 0, 0); break;
    case 1: setLED(ledNumber, 0, ledi, 0); break;
    case 2: setLED(ledNumber, 0, 0, ledi); break;
//     case 3: setLED(ledNumber, 128, 0, 0); break;
//     case 4: setLED(ledNumber, 0, 128, 0); break;
//     case 5: setLED(ledNumber, 0, 0, 128); break;
    case 3: setLED(ledNumber, ledi, ledi, ledi); break;
    default: 
      color = 0; 
      ledi *= 2;
      if (ledi == 0)
        ledi = 1;
      ledNumber--;
      break;
  }
}
/**
 * Main loop
 * primarily for initialization,
 * non-real time services and
 * synchronisation with heartbeat.*/
extern "C" int main(void)
{
//   uint32_t loop1 = 0, loop2=0;
  uint32_t loop = 0;
//   const int MSL = 200;
//   char s[MSL];
  // debug main
  digitalWriteFast(LED_BUILTIN,1);  
  // wait a bit - to allow usb to connect in order to see init errors
  delay(1000); // ms
  // 
  initialization();
  setLED(0, 255, 0, 0);
  //
  while (1)
  { // main loop
    // get data from usb - if available
    handleIncoming();
//     loop1++;
    delay(1);
    //
    startNewCycle = true;
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle)
    { // start of new 1ms control cycle
      startNewCycle = false;
      if (newLedValue)
      {
        newLedValue = false;
        show();
      }
      //
      if (loop % 100 == 0)
      {
        changeDisply();
      }
      // blink the on-board LED
      loop++;
      if (loop % 1000 == 0)
        digitalWriteFast(LED_BUILTIN,1);
      else if (loop % 1000 == 100)
        digitalWriteFast(LED_BUILTIN,0);
    }
  }
  return 0;
}



void show()
{
  //   usb_send_str("# show start\r\n");
  led_cycle = true;
  digitalWriteFast(ledPin, 0); 
  delayActualCnt = 0;
  ledBit = 7;
  ledIdx = 0;
  bool first = true;
  cli();
  while (led_cycle)
  { // start with one
    ledBitVal = (ledValues[ledIdx] >> ledBit) & 1;
    digitalWriteFast(ledPin, 1); 
    // wait for 2nd
    if (first)
    {
      first = false;
      delayActualCnt = 10;
      delayActualCnt = 11;
      delayActualCnt = 12;
      delayActualCnt = 13;
      delayActualCnt = 14;
    }
    else
    {
      delayActualCnt = 1;
      delayActualCnt = 2;
      delayActualCnt = 3;
      delayActualCnt = 4;
      delayActualCnt = 5;
      delayActualCnt = 6;
    }
    // stop if a zero
    digitalWriteFast(ledPin, ledBitVal);
    // wait for second stop position
    delayActualCnt = 4;
    delayActualCnt = 5;
    delayActualCnt = 6;
    delayActualCnt = 7;
    delayActualCnt = 8;
    delayActualCnt = 9;
    // stop now if 1
    digitalWriteFast(ledPin, 0);
    // finished with this bit goto next
    ledBit--;
    if (ledBit < 0)
    { // finished with byte
      ledBit = 7;
      ledIdx++;
      if (ledIdx >= LED_CNT * 3)
      { // finished
        led_cycle = false;
      }
      //           usb_send_str("# new led\r\n");
    }
    else
    { // from 0.6 us to 1.25 us delay
      delayActualCnt = 26;
      delayActualCnt = 27;
      delayActualCnt = 28;
      delayActualCnt = 29;
    }
    delayActualCnt = 10;
    delayActualCnt = 11;
    delayActualCnt = 12;
    delayActualCnt = 13;
    delayActualCnt = 14;
    delayActualCnt = 15;
    delayActualCnt = 16;
    delayActualCnt = 17;
    delayActualCnt = 18;
    delayActualCnt = 19;
    delayActualCnt = 20;
    delayActualCnt = 21;
  }
  sei();
}

