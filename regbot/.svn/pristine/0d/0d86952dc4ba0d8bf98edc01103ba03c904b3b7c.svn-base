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

#include <sys/time.h>
#include <mutex>

#include "urun.h"
#include "utime.h"
#include "ulogfile.h"
#include "tcpCase.h"

class UHandler;


using namespace std;

/**
 * The robot class handles the 
 * port to the REGBOT part of the robot,
 * REGBOT handles the most real time issues
 * and this class is the interface to that. */
class UTeensy : public URun
{ // REGBOT interface
public:
  /// Is port opened successfully
  bool teensyConnectionOpen;
  // mission state from hbt 
  int missionState = 0;
  
  
private:
  // serial port handle
  int usbport;
  // serial port (USB)
  const char * usbdevice = "/dev/ttyACM0";
  // simulator hostname
  const char * simHost;
  // simulator port
  int simPort = 0;
  // mutex to ensure commands to regbot is not mixed
  mutex sendMtx;
//   mutex logMtx;
  mutex eventUpdate;
  // receive buffer
  static const int MAX_RX_CNT = 1000;
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt;
  // command and data handler
  UHandler * handler;
  //
  UTime lastTxTime;
  // socket to simulator
  tcpCase socket;
  
public:
  /** constructor */
    UTeensy();
  /** destructor */
    ~UTeensy();
  /**
   * Set device */
  void setDevice(char * usbDev, int simport, char * simhost)
  {
    usbdevice = usbDev;
    simHost = simhost;
    simPort = simport;
  }
  /**
   * send a string to the serial port 
   * But wait no longer than timeout - the unsend part is skipped 
   * \param key is message ID,
   * \param params is message parameters
   * \param timeoutMs is number of ms to wait at maximum */
  void sendMsg(const char * key, const char * params, int timeoutMs = 10);
  /**
   * send a string to the serial port 
   * But wait no longer than timeout - the unsend part is skipped 
   * \param cmd is c_string to send,
   * \param timeoutMs is number of ms to wait at maximum */
  void sendString(const char * cmd, int timeoutMs = 10);
  /**
   * runs the receive thread 
   * This run() function is called in a thread after a start() call.
   * This function will not return until the thread is stopped. */
  void run();
  /**
   * Set pointer to cladd that handles all messsaged 
   * and start read thread */
  void setHandler(UHandler * dataHandler);
  /**
   * Init data types to and from robot */
  void initMessageTypes();
  /**
   * open log with communication with robot */
  void openCommLog(const char * path);
  /** close logfile */    
  void closeCommLog();
  
private:
  /**
   * Open the connection.
   * \returns true if successful */
  bool openToTeensy();
  /**
   * decode messages from REGBOT */
  void decodeFromTeensy(char * msg);
  /**
   * Logvile */
  ULogFile * botComTx;
  ULogFile * botComRx;
  //   mutex logMtx;
  int connectErrCnt = 0;
};

#endif
