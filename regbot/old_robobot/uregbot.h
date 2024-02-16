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

#ifndef UREGBOT_H
#define UREGBOT_H

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
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "urun.h"

using namespace std;

/**
 * The robot class handles the 
 * port to the REGBOT part of the robot,
 * REGBOT handles the most real time issues
 * and this class is the interface to that. */
class URegbot : public URun
{ // REGBOT interface
public:
  // status from regbot
  /// last known pose
  float x,y,h;
  /// distance traveled (always positive)
  float dist;
  // regbot time
  float regbotTime;
  timeval bootTime;
  float batteryVoltage;
  bool isConnected;
  // mission running
  bool missionRunning;
  // edge detect
  bool edgeValidLeft;
  bool edgeValidRight;
  bool edgeCrossingBlack;
  bool edgeCrossingWhite;
  // edge position in [cm], 0=center (or not valid)
  float edgeRight;
  float edgeLeft;
  /**
   * event number */
//   int eventNumber;
  /**
   * Event flags captures if more than one event at a time */
  static const int MAX_EVENT_FLAGS = 34;
  bool eventFlags[MAX_EVENT_FLAGS];
  /// log for communication
  FILE * botComLog;
  /// log for data, with timestamp as received by pi
  FILE * botDataLog;
  /// IR distance in cm for sensors
  float irDist[2];
  
private:
  // serial port handle
  int port;
  // serial port (USB)
  const char * usbport = "/dev/ttyACM0";
  // read thread handle
  thread * th1;
  // set true to stop thread
  bool th1stop;
  // mutex to ensure commands to regbot is not mixed
  mutex sendMtx;
  mutex logMtx;
  mutex eventUpdate;
  // receive buffer
  static const int MAX_RX_CNT = 500;
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt;
  // status message sequence (fast (sensors))
  int statusMsgIdx;
  // status message sequence (slow ("static" info))
  int statusMsgIdx2;
  
public:
  /** constructor */
  URegbot();
  /** destructor */
  ~URegbot();
  /**
   * send a string to the serial port */
  void send(const char * cmd);
  /**
   * receive thread */
  void run();
  /**
   * clear events */
  void clearEvents();
  /**
   * test and reset event */
  bool eventSet(int event);
  /**
   * Set an event flag
   * \param eventNumber is event to set (in range 0..33) */
  void setEvent(int eventNumber);
  /**
   * stop interface */
  void stop();
  /**
   * Get time since boot in float seconds */
  float getTime()
  {
    timeval t;
    gettimeofday(&t, NULL);
    return getTimeDiff(t, bootTime);
  }
    
private:
  /**
   * Open the connection.
   * \returns true if successful */
  bool openToRegbot();
  /**
   * decode messages from REGBOT */
  void decode(char * msg);
  /** decode event message */
  void decodeEvent(char * msg);
  /** decode heartbeat message */
  void decodeHbt(char * msg);
  /** decode heartbeat message */
  void decodePose(char * msg);
  /** decode line sensor edge message */
  void decodeEdge(char * msg);
  /** decode IR distances */
  void decodeIR(char * msg);
  /** decode mission status - running or not */
  void decodeMissionStatus(char * msg);
  /** send regulat status requests, if no other traffic */
  void sendStatusRequest();
  /** save pose and other data to log */
  void saveDataToLog();
};

#endif
