/*
 * respond functions to data items ment for bridge
 * 
 ***************************************************************************
 *   Copyright (C) 2017 by DTU (Christian Andersen)                        *
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

#ifndef UBRIDGE_H
#define UBRIDGE_H

#include "urun.h"
#include "utime.h"

class UDataItem;
class UTeensy;
class UOled;
class UData;
class UServerPort;
class UHandler;

// shutdown and restart flags
extern bool quitBridge;
extern bool restartBridge;

class UBridge : URun
{
public:
  /**
   * constructor */
  UBridge(UTeensy * robot, UOled * display, UData * data, UServerPort * portServer);
  /** destructor */
  ~UBridge();
  /** get responce number 
   * searches if this data item needs some special treatment,
   * such as send to oled display or send something to robot
   * \param key is data item key,
   * \param client is the message source client number (robot is -1, joystick=-2)
   * \param sequence is to return to data item, if this datatype
   *                 is a sequence (e.g. logfile or program list
   * \returns number of responce process, or -1 if no special
   *          process is needed. */
  int getResponceNumber(const char * key, int client, bool * sequence);
  /**
   * Responce function with pointer to data item 
   * \param respondceNumber is allocated responce function (allocated by the getResponceNumber() function
   * \param dataItem is pointer to updated data item 
   * \param client is the source number for the data update */
  void responce(int responceNumber, UDataItem * dataItem, int client);
  /**
   * Set message handler */
  void setHandler(UHandler * messageHandler);
  /**
   * bridge thread loop */
  void run();
  /**
   * time for last hbt */
  UTime lastHbt;
  /**
   * Get pointer to Regbot connection */
  UTeensy * getRegbot()
  {
    return bot;
  }
  
private:
  /**
   * Responce proocess */
  void responseParamToRobot(UDataItem * dataItem, int client);
  /**
  * Send both key and parameter to robot */
  void responceAllToRobot(UDataItem* dataItem);
  /**
   * Is for oled display */
  void responceStatus(UDataItem * dataItem);
  /**
   * Is for this bridge itself */
  void responceBridge(UDataItem * dataItem, int client);
  /**
   * Echo data item to console */
  void responceConsole(UDataItem * dataItem, int client);
  
  /**
   * information from socket server */
  void responceClient(UDataItem* dataItem, int client);
  /**
   * information from socket server */
  void responceRobotID(UDataItem* dataItem, int client);
  /** measure CPU temperature
   * */
  float measureCPUtemp();
private:
  // link to other parts of bridge
  UTeensy * bot;
  UOled * oled;
  UData * data;
  UServerPort * serv;
  UHandler * handler;
  //
  bool listItems = false;
  int  listItemIterator = 0;
  int listClient;
  bool keepHbt = true;
  bool getStatusFromRegbot;
  //
};


#endif
