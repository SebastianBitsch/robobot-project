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
#define REV "$Rev: 353 $"
#define REV_MINOR 3

#define REV_ID "$Id: main.cpp 353 2016-07-12 15:38:03Z jcan $" 

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

float time = 0.0; // system time in seconds
uint32_t timeAtMissionStart = 0;
const char * lfcr = "\n\r";
bool pushToUSB = 1;
bool logToUSB = 0;
// should teensy echo commands to usb
int8_t localEcho = 0;
uint32_t motorCurrentMLowPass[2];
//
int pushInterval = 0;
int pushTimeLast = 0;
int pushStatus = 0;
// line sensor full or reduced power (output is high power)
bool pinMode18 = OUTPUT;
//
// mission stop and start
bool button;
uint16_t buttonCnt;
bool missionStart = false;
bool missionStop = false;
bool sendStatusWhileRunning = false;
/**
 * usb command buffer space */
const int RX_BUF_SIZE = 110;
char usbRxBuf[RX_BUF_SIZE];
int usbRxBufCnt = 0;

int * m1;
/**
 * write to I2C sensor (IMU) */
int writeSensor(uint8_t i2caddr, int reg, int data);
/**
 * send state as text to USB (direct readable) */
void stateToUsb();
/// send push status
void sendStatusSensor();
//void sendStatusLogging();
void sendStatusVersion();
/// eeprom functions
void eePromSaveStatus(uint8_t * configBuffer);
void eePromLoadStatus(const uint8_t * configBuffer);
///
///
int imuReadFail = 0;
/** Who is requesting data
 * -2 = none (push), -1=USB, 0..4 = wifi client */
int8_t requestingClient = -2;

/**
 * Get SVN revision number */
uint16_t getRevisionNumber()
{
  return strtol(&REV[5], NULL, 10) * 100 + REV_MINOR;
}

////////////////////////////////////////////

void sendStatusLS()
{
  const int MRL = 250;
  char reply[MRL];
  //                       #1 #2   LS0    LS1    LS2    LS3    LS4    LS5    LS6    LS7    timing (us)
  snprintf(reply, MRL, "ls %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %ld %ld\r\n",
           adcStartCnt, adcHalfCnt, 
           adcLSH[0], adcLSL[0], adcLSH[1], adcLSL[1], adcLSH[2], adcLSL[2], adcLSH[3], adcLSL[3], 
           adcLSH[4], adcLSL[4], adcLSH[5], adcLSL[5], adcLSH[6], adcLSL[6], adcLSH[7], adcLSL[7], 
           adcConvertTime * 10, adcHalfConvertTime * 10);
  usb_send_str(reply);
}

////////////////////////////////////////////

void sendStatusCurrentVolt()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "VA %d %.3f  %d %d %.3f  %d %d %.3f\r\n",
           batVoltInt, batVoltInt * batVoltIntToFloat, 
           motorCurrentM[0], motorCurrentMOffset[0], getMotorCurrentM(0, motorCurrentM[0]),
           motorCurrentM[1], motorCurrentMOffset[1], getMotorCurrentM(1, motorCurrentM[1])
          );
  usb_send_str(reply);
}

//////////////////////////////////////////////////////

/**
 * Test all I2C adresses and print reply. */
void testAddr(void)
{
  int ak;
  const int MRL = 100;
  char reply[MRL];
  for (int i= 0; i < 0x7f; i++)
  {
    Wire.beginTransmission(i);
    Wire.write(0);
    ak = Wire.endTransmission(I2C_STOP,1000);
    snprintf(reply, MRL, "addr test addr %d (%x) gave %d", i, i, ak);
    usb_send_str(reply);
  }
}

// ////////////////////////////////////////

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
        if (strncmp(usbRxBuf, "alive", 5) == 0 or strncmp(usbRxBuf, "<alive", 6) == 0)
        { // just an alive signal - reply with mobotware message
          const int MSL = 50;
          char s[MSL];
          snprintf(s, MSL, "<alive last=\"%.5f\"/>\r\n", float(controlUsedTime[1]) / 100000.0);
          usb_send_str(s);
        }
        else
          parse_and_execute_command(usbRxBuf, -1);
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

/**
 * Read data from sensors, that is the relevant sensors
 * \returns true if IMU could be read */
void readSensors()
{
  bool b;
//   #define AnalogA0 15 // motor 1 (left)
//   #define AnalogA1 14 // motor 2
  if (lsPowerHigh and pinMode18 == INPUT)
  { // high power mode - use both pin 18 and pin 32
    pinMode(18,OUTPUT); // Line sensor power control 
    pinMode18 = OUTPUT;
  }
  else if (not lsPowerHigh and pinMode18 == OUTPUT)
  { // low power mode - use pin 32 only (or pin 25 when power board is installed)
    pinMode(18,INPUT); // Line sensor power control
    pinMode18 = INPUT;
  }
  if (useADCInterrupt)
  { // start first AC conversion
    adcSeq = 0;
    adcHalf = false;
    /*adcErr0[adcSeq] =*/ adc->startSingleRead(adcPin[0], -1); // + 400;
    adcStartTime = hb10us;
    adcStartCnt++;
  }
  // read start button
  if (robotId >15)
    // mainboard version 2B
    b = digitalRead(6);
  else
    // mainboard version 1A
    b = digitalRead(11);
  // prell prohibit
  if (b)
  { // butten pressed
    if (buttonCnt == 49)
    {
      button = true;
      // debug 
      // stopTeensy();
      // debug end
    }
    if (buttonCnt < 50)
    buttonCnt++;
  }
  else
  { // button not pressed
    if (buttonCnt == 1)
      button = false;
    if (buttonCnt > 0)
      buttonCnt--;
  }
  //
  if (imuAvailable)
  { // read from IMU
    //
    // NB! this takes 480us to read from gyro
    // this should be using interrupt !!!!!!!!!!!!!!!!
    //readAccGyro();
    if (true)
    {
      int isOK;
      isOK = mpuReadData();
      if (not isOK)
      {
        imuReadFail++;
        if (imuReadFail % 32 == 31)
        {
          const int MSL = 70;
          char s[MSL];
          snprintf(s, MSL, "# failed to get IMU data, got = %d (tried %d times)\r\n", isOK, imuReadFail);
          usb_send_str(s);
        }
      }
      else
        imuReadFail = 0;
      isOK = mpuRequestData();
      if (not isOK)
        usb_send_str("# failed to request IMU data\r\n");
    }
    else
    {
      int a = readAccGyro();
      if (a != 0)
        usb_send_str("#ACCGyro read failed\r\n");
    }      
  }
  //
  // start read of analogue values
  if (useADCInterrupt)
  { // convert last value to float
    // measured over a 15kOhm and 1.2kOhm divider with 1.2V reference
    //batVoltFloat = batVoltInt * 1.2 / useADCMaxADCValue * (15.0 + 1.2)/1.2;
    //
    if (motorPreEnabled)
    { // low pass input values (using long integer) - about 100ms time constant (if currentCntMax==1)
      // running until first motor is enabled
      motorCurrentMLowPass[0] = (motorCurrentMLowPass[0] * 199)/200 + motorCurrentM[0];
      motorCurrentMLowPass[1] = (motorCurrentMLowPass[1] * 199)/200 + motorCurrentM[1];
      // save as direct usable value
      motorCurrentMOffset[0] = motorCurrentMLowPass[0] / 200;
      motorCurrentMOffset[1] = motorCurrentMLowPass[1] / 200;
    }
    motorCurrentA[0] = getMotorCurrentM(0, motorCurrentM[0]);
    motorCurrentA[1] = getMotorCurrentM(1, motorCurrentM[1]);
  }
  else
  {
//     if (--currentCnt <= 0)
    { // time to read motor current
      motorCurrentM[0] = analogRead(10); // pin A10 and A11
      motorCurrentM[1] = analogRead(11);
      if (motorPreEnabled)
      { // low pass input values (using long integer) - about 100ms time constant (if currentCntMax==1)
        motorCurrentMLowPass[0] = (motorCurrentMLowPass[0] * 99)/100 + motorCurrentM[0];
        motorCurrentMLowPass[1] = (motorCurrentMLowPass[1] * 99)/100 + motorCurrentM[1];
        // save as direct usable value
        motorCurrentMOffset[0] = motorCurrentMLowPass[0] / 100;
        motorCurrentMOffset[1] = motorCurrentMLowPass[1] / 100;
      }
//      currentCnt = currentCntMax;
    }
    if (adcSeq < 8)
    {
      adcLSH[adcSeq] = analogRead(adcPin[adcSeq + 3]);
      adcSeq++;
    }
    else
      adcSeq = 0;
  }
  //if (--batVoltCnt <= 0)
  { // read voltage from pin 23 - scaled by 15k over 3.6k and Vref=3.3
    if (not useADCInterrupt)
    {
      batVoltInt = analogRead(23);
    }
  }
}

//////////////////////////////////////////


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
  // handling og wifi data
  // get one character
  for (int i = 0; i < 10; i++)
  { // read up to 10 chars from wifi connections
    n = Serial1.read();
    if (n < 0)
      break;
    if (n > '\0' and n < 0x80)
    { // potentially usable character
      char * cmd;
      int channel;
      // debug relay wifi input to USB
      //usb_serial_putchar(n);
      // debug end relay wifi input to USB
      if (wifi.receivedCharFromSer(n, &cmd, &channel))
      {
        parse_and_execute_command(cmd, channel);
      }      
  //     const int MSL = 70;
  //     char s[MSL];
  //     snprintf(s, MSL, "# handleIncoming got %d chars, wait=%d\r\n", serRxBufCnt, wifiWaitForSendOK);
  //     usb_send_str(s);    
    }
  }
  if (wifi.sendingState == UWifi8266::WFD_SEND_REQ or wifi.sendingState == UWifi8266::WFD_SENDING)
  {
    if (hbTimerCnt - wifi.waitForSendOKtime > 1000)
    { // debug/error?
//       const int MSL = 100;
//       char s[MSL];
//       snprintf(s, MSL, "# %lu ms wifi timeout - not send %s... msg\r\n", hbTimerCnt, wifi.serTxBuf);
//       usb_send_str(s);
      // debug/err end
      wifi.sendingState = UWifi8266::WFD_SEND_FAILED;
    }
  }  
}

//////////////////////////////////////////


void sendHartBeat()
{
  const int MRL = 35;
  char reply[MRL];
  snprintf(reply, MRL,  "hbt %g\r\n", time);
  usb_send_str(reply);
}

void sendStatusSensor()
{
  //int switch2 = digitalRead(11);
  const int MRL = 350;
  char reply[MRL];
  //   float mc0 = getMotorCurrent(0);
  //   float mc1 = getMotorCurrent(1);
  snprintf(reply, MRL,"acw %g %g %g\n"
                        "gyw %g %g %g\n"
                        "gyo %g %g %g %d\n"
                        "mca %g %g\n" //%d %d %d %d %ld %ld %g %g\n"
                        "enc 0x%lx 0x%lx\n"
                        "wve %g %g\n"
                        "wpo %g %g\n"
                        "pse %g %g %g %g %g\n"
                        "swv %d\n"
                        "bat %g\r\n" ,
          imuAcc[0] * accScaleFac, imuAcc[1] * accScaleFac, imuAcc[2] * accScaleFac,
          imuGyro[0] * gyroScaleFac, imuGyro[1] * gyroScaleFac, imuGyro[2] * gyroScaleFac,
           offsetGyro[0] * gyroScaleFac, offsetGyro[1] * gyroScaleFac, offsetGyro[2]* gyroScaleFac, gyroOffsetDone,
//            mc0, mc1, motorCurrentM[0], motorCurrentM[1], 
//            motorCurrentMOffset[0], motorCurrentMOffset[1],
//           motorCurrentMLowPass[0], motorCurrentMLowPass[1],
           getMotorCurrentM(0, motorCurrentM[0]), getMotorCurrentM(1, motorCurrentM[1]),
          encoder[0], encoder[1], 
          wheelVelocityEst[0], wheelVelocityEst[1],
          wheelPosition[0], wheelPosition[1],
          pose[0], pose[1], pose[2], distance, pose[3],
          button, 
          batVoltInt * batVoltIntToFloat
    );
  usb_send_str(reply);
}



void sendStatusVersion()
{
  const int MRL = 100;
  char reply[MRL];
  const char * p1 = REV;
  snprintf(reply, MRL, "version %d.%ld.%d %d\r\n", REV_MAIN,  strtol(&p1[5], NULL, 10), REV_MINOR, imuAvailable);
  usb_send_str(reply);
}


// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(char *buf, int8_t serChannel)
{ // first character is the port letter
  const int MSL = 100;
  char s[MSL];
  char * p2;
  //
  requestingClient = serChannel;
  if (buf[1] == '=')
  { // one character assignment command
    bool isOK;
    switch (buf[0])
    {
      case 'i':  // interactive mode -1=no info, 0=normal GUI, 1=telnet session (local echo of all)
        localEcho = strtol(&buf[2], NULL, 10);
        break;
      case 'M':  // mission to run
        if (missionState == 0)
        { // mission can not be changed while another mission is running
          mission = strtol(&buf[2], NULL, 10); 
          if (mission >= missionMax)
            mission = missionMax - 1;
        }
        else
          usb_send_str("#* sorry, a mission is in progress.\r\n");
        break;
      case 's':  // status interval
        logInterval = strtol(&buf[2], &p2, 10);
        if (*p2 >= ' ')
          logAllow = strtol(p2, &p2, 10); 
        break;
      case 'S':  pushInterval = strtol(&buf[2], NULL, 10); break;
      case 'R':  sendStatusWhileRunning = strtol(&buf[2], NULL, 10); break;
      //case 'n':  pushCnt = strtol(&buf[2], NULL, 10); break;
      //case 'm':  magCntMax = strtol(&buf[2], NULL, 10); break;
      case 'a':  
        usb_send_str("# trying to access IMU: sending request\r\n");
        isOK = mpuRequestData();
        if (isOK)
          usb_send_str("# sending request was OK\r\n");
        else
          usb_send_str("# sending request failed\r\n");
        mpuReadData();        
        break;
      //case 'c':  currentCntMax = strtol(&buf[2], NULL, 10); break;
      case 'w':
        if (serChannel >= 0 and serChannel < UWifi8266::WIFI_MAX_CLIENTS)
        { // wifi connections only
          int n = strtol(&buf[2], NULL, 10);
          switch (n) 
          {
            case 0: wifi.clientActive[serChannel] = UWifi8266::WIFI_NOT; break;
            case 1: wifi.clientActive[serChannel] = UWifi8266::WIFI_MUST; break;
            case 2: wifi.clientActive[serChannel] = UWifi8266::WIFI_NO_HASH; break;
            case 3: wifi.clientActive[serChannel] = UWifi8266::WIFI_ALL; break;
          }
        }
        break;
      default: 
        usb_send_str("#** unknown command!\r\n");
        break;
    }
  }
  else if (setRegulator(buf))
  { // regulator parameters
  }
  else if (setLineSensor(buf))
  { // line sensor settings
  }
  else if (setIrCalibrate(buf))
  { // infrared distance sensor data
  }
  else if (buf[0] == '<')
  { // mission line
    if (strncmp(&buf[1], "clear", 5) == 0)
      userMission.clear();
    else if (strncmp(&buf[1], "add", 3) == 0)
    {
      userMission.addLine(&buf[4]);
    }
    else if (strncmp(&buf[1], "event", 5) == 0)
    {
      userMission.decodeEvent(&buf[6]);
    }
    else if (strncmp(&buf[1], "mod", 3) == 0)
    { // modify line
      int thread = strtol(&buf[4], &p2, 10);
      bool isOK = p2 != &buf[4] and p2 != NULL;
      int line   = strtol(p2, &p2, 10);
      isOK &= p2 != &buf[4] and p2 != NULL;
      if (isOK and thread >= 0 and line>=0)
      { // has got valid thread and line numbers
        isOK = userMission.modLine(thread, line, p2);
      }
      if (not isOK)
        usb_send_str("# missing line or thread number or other error\r\n");
    }
    else if (strncmp(&buf[1], "get", 3) == 0)
    {
      userMission.getLines();
      //usb_send_str("# send all lines?\n");
    }
    else if (strncmp(&buf[1], "token", 5) == 0)
    {
      userMission.getToken();
    }
    else
      usb_send_str("# no such user mission command\r\n");
  }
  else if (buf[0] == 'u')
  { // reply on user data request
    int8_t r = strtol(&buf[1], NULL, 10);
    switch (r)
    {
      case 0: sendStatusVersion(); break;
      case 1: 
        //sendHartBeat();
        sendStatusSensor();  break;
      case 2: 
        sendMissionStatusChanged(true); break;
      case 3: sendStatusLogging();  break;
      case 4: sendStatusRobotID(); break;
      //case '5': sendStatusControl(-1); break;
      case 6: sendStatusMag(); break;
      case 7: sendStatusLS(); break;
      case 8: sendStatusCurrentVolt(); break;
      case 9:  // ADC values
      case 10: // limits
      case 11: // values and found edges
      case 12: // LS gain
        sendADLineSensor(r - 8); 
        break;
      default:
        usb_send_str("#** unknown U status request!\r\n");
        break;
    }
  }
  else if (buf[0] == 'v')
  { // reply on user data request
    switch (buf[1])
    {
      case '0': sendStatusDistIR(); break;
      case '1': wifi.sendStatusWiFi(); break;
      case '2': wifi.sendStatusWiFiClients(); break;
      default:
        usb_send_str("#** unknown V status request!\r\n");
        break;
    }
  }
  else if (buf[0] == 'x')
  { // reply on user data request
    int8_t r = strtol(&buf[1], NULL, 10);
    sendStatusControl(r);
  }
  else if (strncmp(buf, "halt", 4) == 0)
  { // stop all 12V power (or turn on if 12 V power is off (and switch is on))
    if (buf[4] >= ' ')
      batteryHalt = strtol(&buf[5], NULL, 10);
    else
      batteryHalt = true;
  }
  else if (buf[0] == 'h' || buf[0] == 'H')
  {
    const int MRL = 200;
    char reply[MRL];
    snprintf(reply, MRL, "# RegBot commands (" REV_ID ")\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   M=N    Select mission 0=motor test, 1=turn test, 2=Balance, 3=user, 4=SS, 5=linefollow (M=%d)\r\n", mission);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   i=1    Interactive: 0: GUI info, 1: use local echo of all commands, -1:no info  (i=%d)\r\n", localEcho);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   s=N A  Log interval in milliseconds (s=%d) and allow A=1 (is %d)\r\n", logInterval, logAllow);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   log+/-item  Log item (mis %d, acc %d, gyro %d, " /*"mag %d, " */ "motR %d, "
                               "motV %d, motA %d, enc %d, mvel %d, tr %d, pose %d, line %d, dist %d, bat %d, bcl %d, ct %d)\r\n",
             logRowFlags[LOG_MISSION], logRowFlags[LOG_ACC], logRowFlags[LOG_GYRO], //logRowFlags[LOG_MAG],
             logRowFlags[LOG_MOTV_REF], logRowFlags[LOG_MOTV], 
             logRowFlags[LOG_MOTA], logRowFlags[LOG_ENC], logRowFlags[LOG_WHEELVEL], logRowFlags[LOG_TURNRATE], logRowFlags[LOG_POSE], 
             logRowFlags[LOG_LINE], logRowFlags[LOG_DIST],
             logRowFlags[LOG_BATT], logRowFlags[LOG_BAL_CTRL], logRowFlags[LOG_CTRLTIME]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   log start   Start logging to %dkB RAM (is=%d, logged %d/%d lines)\r\n", LOG_BUFFER_MAX/1000, loggerLogging(), logRowCnt, logRowsCntMax);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   log get     Transfer log to USB (active=%d)\r\n", logToUSB);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   motw m1 m2   Set motor PWM -1024..1024 (is=%d %d)\r\n", motorAnkerPWM[0], motorAnkerPWM[1]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   motv m1 m2   Set motor voltage -6.0 .. 6.0 (is=%.2f %.2f)\r\n", motorAnkerVoltage[0], motorAnkerVoltage[1]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   mote m1 m2   Set motor enable (left right) (is=%d %d)\r\n", motorEnable[0], motorEnable[1]);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   uX           Status: u0:ver,u1:measure,u2:mission, u3:log,u4:robot,u6:mag,u8:motorVA,u9,u10,u11,u12:LS all\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   vX           Status: v0:IR sensor value, v1:wifi status, v2 wifi clients\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   xX           Status control: x1:turn,x2:edge,X3:wall,x4:vel,x5:pos,x6:Zvel+turn,x7:Zedge,x8:Zpos,x9:Zwall,x10:bal,x11:SS,x12:balvel\r\n");
    usb_send_str(reply);
    usb_send_str(        "#   rX=d d d ... Settingsfor regulator X - use GUI or see code for details\r\n");
    snprintf(reply, MRL, "#   rid=d d d d d d d  Robot ID set: ID wheelBase gear PPR RadLeft RadRight balanceOffset\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eew          Save configuration to EE-Prom\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eeW          Get configuration as string\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eer          Read configuration from EE-Prom\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   eeR=X        Read config and mission from hard coded set X=0: empty, X=1 follow wall\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   S=N          Push status every N milliseconds (is %d)\r\n", pushInterval);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   R=N          Push status while running 1=true (is %d)\r\n", sendStatusWhileRunning);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   w=N          wifi msg filter 0=nothing, 1=must (log), 2=must+status, 3=all\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   posec        Reset pose and position\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   gyroo        Make gyro offset calibration\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   mem          Some memory usage info\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   start   start mission (and logging) now\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   stop   terminate mission now\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <add user-mission-line>  add a user mission line (%d lines loaded in %d threads)\r\n", 
             userMission.getLinesCnt(), userMission.getThreadsCnt());
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <clear>      Clear all user mission lines\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <get>        Get all user mission lines\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <event=X>    Make an event number X (from 0 to 31)\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   <token>      Get all user mission lines as tokens\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   :xxxx        Send data (AT commands) to wifi board (all except the ':') \\r\\n is added\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   link=L,data  Send data to socket link L\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   wifi restart port SSID PW   Wifi setup (e.g. 'wifi 1 24001 \"device\" \"\"')\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   halt         Turn 12V power off (on by halt=0) (is %d)\r\n", batteryHalt);
    usb_send_str(reply);
    snprintf(reply, MRL, "#   alive        alive command - initates an <alive last=\"0.0xxx\"/> reply\r\n");
    usb_send_str(reply);
    snprintf(reply, MRL, "#   help   This help text\r\n");
    usb_send_str(reply);
  }
  else if (strncmp(buf, "eew", 3) == 0)
  { // initalize ee-prom
    eeConfig.setStringBuffer(NULL, true);
    // save to ee-prom
    usb_send_str("# saving to flash\r\n");
    eeConfig.eePromSaveStatus(false);
  }
  else if (strncmp(buf, "eeW", 3) == 0)
  { // save config to string buffer (and do not touch ee-prom)
    uint8_t buffer2k[2048];
    eeConfig.setStringBuffer(buffer2k, false);
    eeConfig.eePromSaveStatus(true);
    // send string-buffer to client
    eeConfig.stringConfigToUSB(NULL, 0);
    eeConfig.clearStringBuffer();
  }
  else if (strncmp(buf, "eer", 3) == 0)
    // load from flash to ccurrent configuration
    eeConfig.eePromLoadStatus(NULL);
  else if (strncmp(buf, "eeR", 3) == 0)
  { // load from hard coded configuration to current configuration
    // get config number
    char * p1 = strchr(buf,'=');
    int m = 1;
    if (p1 != NULL)
    { // skip the equal sign
      p1++;
      // get the configuration number
      m = strtol(p1, NULL, 10);
      if (m < 0)
        m = 0; // factory reset
    }
    // 2. parameter is a debug flag to send 
    // the newly loaded configuration to USB
    eeConfig.hardConfigLoad(m, true);
  }
  else if (strncmp(buf, "stop", 4) == 0)
  {
    missionStop = true;
    logToUSB = false;
  }
//   else if (strncmp(buf, "test", 4) == 0)
//     processingTimeTest();
  else if (strncmp(buf, "start", 5) == 0)
    missionStart = true;
  else if (strncmp(buf, "posec", 5) == 0)
    clearPose();
  else if (strncmp(buf, "log", 3) == 0)
  { // add or remove
    if (strstr(&buf[3], "start") != NULL)
    { // if not mission timing, then zero here
      if (missionState == 0)
        time = 0.0;
      startLogging(0, true);
      logAllow = true;
    }
    if (strstr(&buf[3], "stop") != NULL)
    { // 
      stopLogging();
    }
    else if (strcasestr(&buf[3], "get") != NULL)
    {
      logToUSB = true;
      if (logRowCnt == 0)
        usb_send_str("% log is empty\r\n");
    }
    else
    {
      int plus = buf[3] == '+';
      if (strncasecmp(&buf[4], "mis", 3) == 0)
        logRowFlags[LOG_MISSION] = plus;
      else if (strncasecmp(&buf[4], "acc", 3) == 0)
	logRowFlags[LOG_ACC] = plus;
      else if (strncasecmp(&buf[4], "gyro", 4) == 0)
	logRowFlags[LOG_GYRO] = plus;
      else if (strncasecmp(&buf[4], "motV", 4) == 0)
	logRowFlags[LOG_MOTV] = plus;
      else if (strncasecmp(&buf[4], "motR", 4) == 0)
        logRowFlags[LOG_MOTV_REF] = plus;
      else if (strncasecmp(&buf[4], "motA", 4) == 0)
	logRowFlags[LOG_MOTA] = plus;
      else if (strncasecmp(&buf[4], "enc", 3) == 0)
	logRowFlags[LOG_ENC] = plus;
      else if (strncasecmp(&buf[4], "mvel", 4) == 0)
        logRowFlags[LOG_WHEELVEL] = plus;
      else if (strncasecmp(&buf[4], "tr", 2) == 0)
        logRowFlags[LOG_TURNRATE] = plus;
      else if (strncasecmp(&buf[4], "pose", 4) == 0)
	logRowFlags[LOG_POSE] = plus;
      else if (strncasecmp(&buf[4], "line", 4) == 0)
        logRowFlags[LOG_LINE] = plus;
      else if (strncasecmp(&buf[4], "dist", 4) == 0)
        logRowFlags[LOG_DIST] = plus;
      else if (strncasecmp(&buf[4], "bat", 3) == 0)
	logRowFlags[LOG_BATT] = plus;
      else if (strncasecmp(&buf[4], "ct", 2) == 0)
        logRowFlags[LOG_CTRLTIME] = plus;
      else if (strncasecmp(&buf[4], "bcl", 3) == 0)
        logRowFlags[LOG_BAL_CTRL] = plus;
      else if (strncasecmp(&buf[4], "extra", 5) == 0)
        logRowFlags[LOG_EXTRA] = plus;
      //
      initLogStructure(100000 / CONTROL_PERIOD_10us);

    }
  } 
  else if (strncmp(buf, "mote", 4) == 0)
  { // motor enable
    uint8_t m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtol(p1, (char**)&p1, 10);
    m2 = strtol(p1, (char**)&p1, 10);
    motorSetEnable(m1, m2);
  }
  else if (strncmp(buf, "motw", 4) == 0)
  {
    int m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtol(p1, (char**)&p1, 10);
    m2 = strtol(p1, (char**)&p1, 10);
    motorSetAnkerPWM(m1, m2);
  }
  else if (strncmp(buf, "motv", 4) == 0)
  {
    float m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtof(p1, (char**)&p1);
    m2 = strtof(p1, (char**)&p1);
    // limit to 6.0 volt
    if (m1 > 6) 
      m1 = 6.0;
    else if (m1 < -6)
      m1 = -6;
    if (m2 > 6) 
      m2 = 6.0;
    else if (m2 < -6)
      m2 = -6;
    motorAnkerVoltage[0] = m1;
    motorAnkerVoltage[1] = m2;
    //addMotorVoltage(m1, m2);
    // transfer to PWM
    motorSetAnchorVoltage();
  }
  else if (strncmp(buf, "rid", 3) == 0)
  { // robot ID and other permanent stuff
    setRobotID(&buf[4]);
  }
  else if (strncmp(buf, "gyroo", 5) == 0)
  {
    gyroOffsetDone = false;
  }
  else if (strncmp(buf,"mem", 3) == 0)
  { // memory usage
    snprintf(s, MSL, "# Main 3 areana=%d (m1=%x &time=%x s=%x)\n", mallinfo().arena, 
             (unsigned int)m1, (unsigned int)&time, (unsigned int)s);
    usb_send_str(s);      
    snprintf(s, MSL, "# Main log buffer from %x to %x\n", (unsigned int)logBuffer, (unsigned int)&logBuffer[LOG_BUFFER_MAX-1]);
    usb_send_str(s);      
    snprintf(s, MSL, "# Mission line takes %d bytes (bool is %d bytes)\r\n", 
             sizeof(UMissionLine), sizeof(bool));
    usb_send_str(s);      
  }
  else if (buf[0] == ':')
  { // send to wifi serial connection
    int n = strlen(buf);
    // remove newline
      while (buf[n-1] < ' ')
        n--;
      // add fresh carriage return and newline
      buf[n++] = '\r';
      buf[n++] = '\n';
      buf[n] = '\0';
      // send to 8266
      Serial1.write(&buf[1]);
      //
      wifi.replyType = UWifi8266::WFI_FINISHED;
  }
  else if (strncmp(buf,"wifi", 4) == 0)
  { // setup new wifi connection
    if (buf[4] == ' ' or buf[4] == '=')
      wifi.decodeWifi(&buf[5]);
    else
      wifi.setup = 1;
  }
  else if (strncmp(buf,"link", 4) == 0)
  { // setup new wifi connection
    if (buf[4] == ' ' or buf[4] == '=')
    {
      char * p1 = &buf[5];
      int v = strtol(p1, &p1, 10);
      if (*p1 == ' ' or *p1 == ',')
      {
        p1++;
        wifi.wifiSend(v, p1, true);
        // debug
        snprintf(s, MSL, "# parse link : send to link %d :'%s'\r\n", v, buf);
        usb_send_str(s);
        // debug end
      }
      // debug
      else
      {
        snprintf(s, MSL, "# parse link : send to link %d failed:'%s'\r\n", v, buf);
        usb_send_str(s);
      }
      // debug end
    }
    // debug
    else
    {
      snprintf(s, MSL, "# parse link failed :'%s'\r\n", buf);
      usb_send_str(s);
    }
    // debug end
  }
  else if (strlen(buf) > 1)
  {
    snprintf(s, MSL, "#** unknown %d char command '%s'\r\n", strlen(buf),  buf);
    usb_send_str(s);
    usb_serial_flush_input();
  }
}

/////////////////////////////////////////////////

bool usb_send_str(const char * str, bool blocking) //, bool toUSB, bool toWifi)
{
  int n = strlen(str);
  bool okSend = true;
  if (localEcho == -1 and str[0] == '#')
    // client is a computer and can't read info
    // so ignore such shit
    return true;
  else
  { // a human client, so send all to USB and other clients
    if (requestingClient < 0)
    { // this may give a timeout, and the rest is not send!
      // this happends especially often on lenovo PCs, but they are especially slow in data transfer.
      if (not blocking)
        // surplus characters will be skipped
        usb_serial_write(str, n);
      else
      { // ensure all is send (like log data)
        int k = n, m;
        const char * p1 = str;
        // send as much as possible at a time
        while (k > 0 /*and usbWriteFullCnt < usbWriteFullLimit*/)
        {
          m = usb_serial_write_buffer_free();
          if (m > 0)
          {
            if (m > k)
              m = k;
            usb_serial_write(p1, m);
            k = k - m;
            if (k <= 0)
              break;
            p1 += m;
          }
        }
      }
    }
    if (requestingClient == -2)
    { // a push message - send to first wifi channel only
      if (str[n - 1] == '\n' and n > 4)
      { 
        for (int i = 0; i < UWifi8266::WIFI_MAX_CLIENTS; i++)
        {
          if (wifi.clientActive[i] != UWifi8266::WIFI_NOT)
          {
            if (blocking or wifi.clientActive[i] == UWifi8266::WIFI_ALL or 
              (wifi.clientActive[i] == UWifi8266::WIFI_NO_HASH and str[0] != '#' and not blocking))
            { // send if available time, or wait if blocking is requested
              okSend = wifi.wifiSend(i, str, n);
              // can send to one client at a time only
              break;
            }
//             else
//             {
//               const int MSL = 50;
//               char s[MSL];
//               requestingClient = -1;
//               snprintf(s, MSL, "# client %d has state %d\r\n", i, wifi.clientActive[i]);
//               usb_send_str(s);
//             }
          }
        }
      }
    }
    else if (requestingClient >= 0 and requestingClient < UWifi8266::WIFI_MAX_CLIENTS)
    {
      okSend = wifi.wifiSend(requestingClient, str, n);
    }
  }
  return okSend;
}

////////////////////////////////////////////////////////////////

void sendStatus()
{
  if ((missionState == 0 or sendStatusWhileRunning) and gyroOffsetDone)
  { // send status
    pushStatus++;
    if (pushStatus % 2 == 0)
      // sendHartBeat();
      sendStatusSensor();
    else
    { // else one of the slow changing data types
      switch(pushStatus / 2)
      {
        case 0 : 
          if (sendStatusControl(-1) != 0)
            // stay here until all is send
            pushStatus -= 2;
          break;
        case 1 : 
          sendMissionStatusChanged(false); 
          break;
        case 2 : 
          sendStatusLogging(); 
          break;
        case 3 : 
          sendStatusRobotID(); 
          break;
        case 4:
          sendStatusLineSensor();
          break;
        case 5:
          sendStatusDistIR();
          break;
        case 6:
          sendStatusVersion();
          break;
        case 7:
          wifi.sendStatusWiFi();
          break;
        case 8:
          wifi.sendStatusWiFiClients();
          break;
        default:
          pushStatus = 0;
      }
    }
  }
  else if (not gyroOffsetDone and hbTimerCnt % 1000 == 0)
  {
    usb_send_str("# IMU: gyro not calibrated\n");
  }
}
