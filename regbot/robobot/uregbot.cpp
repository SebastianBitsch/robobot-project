/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

// #include <iostream>
#include <sys/time.h>
#include <cstdlib>
// #include <fstream>
// #include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include "uregbot.h"
#include "utime.h"

using namespace std;

/** constructor */
URegbot::URegbot()
{
  th1stop = false;
  th1 = NULL;
  statusMsgIdx = 0;
  missionRunning = false;
  edgeValidLeft = false;
  edgeValidRight = false;
  edgeCrossingBlack = false;
  edgeCrossingWhite = false;
  x = 0;
  y = 0;
  h = 0;
  dist = 0;
  irDist[0] = 0;
  irDist[1] = 0;
  printf("Regbot:: opening seriel port\n");
  isConnected = openToRegbot();
  if (port != -1)
  { // there is a REGBOT port, so start communication thread
    printf("Regbot:: opened seriel port\n");
    th1 = new thread(runObj, this);
  }
  gettimeofday(&bootTime, NULL);
  clearEvents();
  /** open logfiles */
  if (false)
    // communication log with REGBOT including timestamp
    botComLog = fopen("bot_send_log.txt","w");
  else
    // no communication log
    botComLog = NULL;
  // log for pose and other sensor data
  botDataLog = fopen("bot_data_log.txt", "w");
  if (botDataLog != NULL)
  { // make formatting header for import to MATLAB
    UTime t;
    t.setTime(bootTime);
    const int MSL = 50;
    char s[MSL];
    fprintf(botDataLog, "%% data log started at %s\n", t.getDateTimeAsString(s));
    fprintf(botDataLog, "%% data format:\n");
    fprintf(botDataLog, "%% 1: time since start [seconds]\n");
    fprintf(botDataLog, "%% 2-5 Pose x,y [m], heading [radians], distance [m]\n");
    fprintf(botDataLog, "%% 6-7 IR distance (left, right) [m]\n");
    fprintf(botDataLog, "%% 8-9 Edge sensor (left, right) [cm]\n");
    fflush(botDataLog);
  }
}
/** destructor */
URegbot::~URegbot()
{ // stop all activity before close
  stop();
}

void URegbot::stop()
{
  th1stop = true;
  if (th1 != NULL)
    th1->join();
  if (port != -1)
    close(port);
  if (botComLog != NULL)
    fclose(botComLog);
  if (botDataLog != NULL)
    fclose(botDataLog);
  printf("Regbot:: closed usb port\n");
}
/**
  * send a string to the serial port */
void URegbot::send(const char * cmd)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
  sendMtx.lock();
  if (port != -1)
  {
    int n = strlen(cmd);
    write(port, cmd, n);
    // debug
    {
      timeval t;
      gettimeofday(&t, NULL);
      float dt = getTimeDiff(t, bootTime);
      if (botComLog != NULL)
      {
        logMtx.lock();
        fprintf(botComLog, "%6.3f->: %s", dt, cmd);
        logMtx.unlock();
      }
    // printf("%6.3f->: %s", dt, cmd);        
    }
  }
  // 
  usleep(4000); // Short sleep helps communication be more reliable when many command lines are send consecutively.
  sendMtx.unlock();
}
/**
  * receive thread */
void URegbot::run()
{ // read thread for REGBOT messages
  int n = 0;
  rxCnt = 0;
  timeval idleTime;
  gettimeofday(&idleTime, NULL);
  // get robot name
  //send("u4\n");
  while (not th1stop)
  {
    n = read(port, &rx[rxCnt], 1);
    if (n == 1)
    { // got a new character
      rxCnt++;
      if (rx[rxCnt-1] == '\n')
      { // terminate string
        rx[rxCnt] = '\0';
        //printf("#### Received: %s\n",rx);
        decode(rx);
        rxCnt = 0;
	n = 0;
        // reset idle time
        gettimeofday(&idleTime, NULL);
//         printf("*** got:%s\n", rx);
      }
    }
    else if ((n < 0 and errno == EAGAIN) or n == 0)
    {
      timeval t2;
      gettimeofday(&t2, NULL);
      float dt = getTimeDiff(t2, idleTime);
      if (dt > 0.01)
      { // waitet more than 10 ms - do something
        sendStatusRequest();
        idleTime = t2; // not idle anymore
      }
      usleep(1000);
    }
    else if (n < 0)
    {
      perror("Regbot:: port error");
      usleep(100000);
    }
    usleep(200);
  }
  printf("Regbot:: thread stopped\n");
}  

/**
  * Open the connection.
  * \returns true if successful */
bool URegbot::openToRegbot()
{
  port = open(usbport, O_RDWR | O_NOCTTY | O_NDELAY);
  if (port == -1)
    perror("Regbot:: open_port: Unable to open regbot connection");
  else
  { // set connection to non-blocking
    int flags;
    if (-1 == (flags = fcntl(port, F_GETFL, 0)))
      flags = 0;
    fcntl(port, F_SETFL, flags | O_NONBLOCK);    
  }
  
  struct termios options;
  tcgetattr(port, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(port, TCIFLUSH);
  tcsetattr(port, TCSANOW, &options);
  
  
  return port != -1;
}
/**
  * decode messages from REGBOT */
void URegbot::decode(char * msg)
{
  char * p1 = msg;
  // ship whitespace
  while (*p1 <= ' ' and *p1 > '\0')
    p1++;
  // debug
//   printf("Regbot:: got:%s", p1);
  // debug end
  if (*p1 != '\0')
  {
    if (strncmp(p1, "hbt", 3) == 0)
      decodeHbt(p1); // it is a heartbeat message (time and battry voltage)
    else if (strncmp(p1, "pse", 3) == 0)
      decodePose(p1); // it is a pose message
    else if (strncmp(p1, "lip", 3) == 0)
      decodeEdge(p1); // it is a line edge message
    else if (strncmp(p1, "irc", 3) == 0)
      decodeIR(p1); // it is a line edge message
    else if (strncmp(p1, "event", 5)==0)
      decodeEvent(p1);
    else if (strncmp(p1, "mis ", 4)==0)
      ; // mission status skipped
    else if (strncmp(p1, "rid ", 4)==0)
      ; // robot ID skipped
    else if (*p1 == '#')
      // just a message from Regbot
      printf("%s", p1);
    else
    { // not yet supported message - ignore
      // printf("Regbot:: unknown message: %s", msg);
    }
  }
  if (botComLog != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, bootTime);
    logMtx.lock();
    fprintf(botComLog, "%6.3f<-: %s", dt, msg);
    logMtx.unlock();
  }
}

//////////////////////////////////////////////////


void URegbot::setEvent(int eventNumber)
{
  if (eventNumber < MAX_EVENT_FLAGS and eventNumber >= 0)
  { // event 33 is start button, event 0 is misson stop
    eventUpdate.lock();
    printf("# Event received: %d\n", eventNumber);
    eventFlags[eventNumber] = true;
    eventUpdate.unlock();
  }
}


/** decode event message */
void URegbot::decodeEvent(char * msg)
{ // skip the first 3 characters
  char * p1 = &msg[5];
  int eventNumber = strtol(p1, &p1,0);
  setEvent(eventNumber);
}

/** decode heartbeat message */
void URegbot::decodeHbt(char * msg)
{ // skip the first 3 characters
  char * p1 = &msg[3];
  regbotTime = strtof(p1, &p1);
  batteryVoltage = strtof(p1, &p1);
//   printf("# time=%f, bat=%f\n", regbotTime, batteryVoltage);
}
/** decode heartbeat message */
void URegbot::decodePose(char * msg)
{
  char * p1 = &msg[3];
  float x2 = x, y2 = y;
  x = strtof(p1, &p1);
  y = strtof(p1, &p1);
  h = strtof(p1, &p1);
  dist += hypot(x - x2, y - y2);
//   printf("# pose: x=%.3f, y=%.3f, h=%.1f\n", x, y, h*180/M_PI);
  saveDataToLog();
}

void URegbot::saveDataToLog()
{
  if (botDataLog != NULL)
  { // save pose to log
    fprintf(botDataLog, "%.3f " 
        "%.3f %3f %.5f, %.2f "
        "%.3f %.3f %.1f %.1f\n", 
        getTime(),
        x, y, h, dist,
        irDist[0], irDist[1],
        edgeLeft, edgeRight
           );
    fflush(botDataLog);
  }
}


/** decode line sensor edge message */
void URegbot::decodeEdge(char * msg)
{
  /*   snprintf(reply, MRL, "lip %d %d %.4f %d %.4f %d %d %d %d %d %d %d %d\r\n" ,
   *           lineSensorOn, 
   *           lsIsWhite,
   *           lsLeftSide, lsLeftValid,
   *           lsRightSide, lsRightValid, 
   *           mission_line_LeftEdge,  // not visible in client
   *           crossingWhiteLine , crossingBlackLine, crossingWhiteCnt, crossingBlackCnt,
   *           lsPowerHigh, lsPowerAuto
   */
  char * p1 = &msg[3];
  int d = strtol(p1, &p1, 0);
  if (d)
  { // line sensor is on - so read the rest
    d = strtol(p1, &p1, 0);
    edgeLeft = strtof(p1, &p1);
    edgeValidLeft = strtol(p1, &p1, 0);
    if (not edgeValidLeft)
      edgeLeft = 0;
    edgeRight = strtof(p1, &p1); 
    edgeValidRight = strtol(p1, &p1, 0);
    if (not edgeValidRight)
      edgeRight = 0;
    d = strtol(p1, &p1, 0);
    edgeCrossingBlack = strtol(p1, &p1, 0);
    edgeCrossingWhite = strtol(p1, &p1, 0);
    //
    saveDataToLog();
  }  
}

/**
 * decode IR message
 * formatting in Teensy:
 *  snprintf(reply, MRL, "irc %.3f %.3f %d %d %d %d %d %d %d %d\r\n" ,
 *           irDistance[0], irDistance[1],
 *           irRaw[0], irRaw[1],
 *           irCal20cm[0], irCal80cm[0],
 *           irCal20cm[1], irCal80cm[1],
 *           useDistSensor,
 *           distSensorInstalled
 *  );
 */
void URegbot::decodeIR(char * msg)
{
  char * p1 = &msg[3];
  irDist[0] = strtof(p1, &p1);
  irDist[1] = strtof(p1, &p1);
  // the rest is ignored
  saveDataToLog();
}


/** decode mission message */
void URegbot::decodeMissionStatus(char * msg)
{
  char * p1 = &msg[3];
  /*int midx =*/ strtol(p1, &p1, 0);
  int misState = strtol(p1, &p1, 0);
  // when mission is started, mission state is 1 for 1.0 seconds, then 2 when mission is running
  missionRunning = misState == 2;
}
/** send regulat status requests, if no other traffic */
void URegbot::sendStatusRequest()
{
  switch (statusMsgIdx)
  {
    case 0: 
      switch (statusMsgIdx2)
      { // slow messages - send when time
        case 0: send("u4\n"); break; // name
        case 1: send("u5\n"); break; // hbt
        default: statusMsgIdx2 = -1; break;
      }
      statusMsgIdx2++;
      break;
    // these are the messages you want with a fast update rate
    case 1: send("u22\n"); break; // pose
    case 2: send("u13\n"); break; // line sensor
    case 3: send("v0\n"); break; // IR sensor
    //     case 3: send("u2\n"); break; // mission status
    default: statusMsgIdx = -1; break;
  }
  statusMsgIdx++;
}

/**
 * Clear all event flags to false */
void URegbot::clearEvents()
{
  for (int i = 0; i < MAX_EVENT_FLAGS; i++)
    eventFlags[i] = false;
}

/**
 * Requests if this event has occured.
 * \param event the event flag to test
 * \returns true and resets event, if it was set */
bool URegbot::eventSet ( int event )
{
  bool set = false;
  if (event <= MAX_EVENT_FLAGS and event >= 0)
  { // make sure it is not updated between test and set
    eventUpdate.lock();
    set = eventFlags[event];
    eventFlags[event] = 0;
    eventUpdate.unlock();
  }
  return set;
}

