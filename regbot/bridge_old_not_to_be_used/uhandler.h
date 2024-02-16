/*
 * Handling messages to and from clients
 * and to and from regbot
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

#ifndef UHANDLER_H
#define UHANDLER_H



#include <string>
#include <mutex>
#include "udata.h"
#include "userverport.h"
// #include "uregbot.h"
// #include "ujoy.h"

#define CLIENT_ROBOT     -1
#define CLIENT_JOY       -2
#define CLIENT_BRIDGE    -3
#define CLIENT_CONSOLE   MAX_SOCKET_CLIENTS

class UTeensy;
class UJoy;
class UOled;

class UHandler : public URun
{
public:
  /**
   * Constructor */
  UHandler(UTeensy * robot, UData * broker, UJoy * joystick, UOled * oledDisplay);
  
  ~UHandler();
  /**
   * add all valid message types */
  void addDataItems();
  /**
   * thread that ensures messageflow */
  void run();
  /**
   * Handle socket commands */
  void handleCommand(int client, char* msg, bool recursive);
  
public:
  UTeensy * bot;
  UData * items;
  UJoy * joy;
  UOled * oled;
protected:
  
private:
  /**
   * for message handler */
  std::mutex handlerLock;
  
};

#endif
