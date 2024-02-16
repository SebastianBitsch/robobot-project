/*
 * Data storage (message storage)
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

#ifndef UDATA_H
#define UDATA_H

#include <string>

#include "urun.h"
#include "userverport.h"
#include "userverclient.h"
#include "ulogfile.h"

//#define MAX_PRIORITY 5

// maximum allowed message number
#define MAX_MESSAGE_COUNT 100

//#define MAX_ITEM_KEY_LENGTH 5

class UDataItem;
class UBridge;

class UData : public URun
{
public:
  /**
   * constructor */
  UData(UServerPort * port);
  /**
   * destructor */
  ~UData();
  /**
   * new message for a data item from this client
   * \param key ID for this message,
   * \param params string with remaining parameters
   * \param client is the sourse for the message
   * */
  void updateItem(const char * key, char * params, int client);
  
//   void setPriority(int client, int newPriority, int itemIndex);
  
  void setItem(int index, std::string itemName, const char * itemKey);
  /**
   * print data status to console */
  void printStatus();
  /**
   * Publish meta information of a data item */
  int publishItemMeta(int item, int client);
  /**
   * get number of items in database */
  inline int getItemCnt()
  {
    return itemsCnt;
  }
  /**
   * set pointer to responce handler */
  void setHandler(UHandler * messageHandler, UBridge * responsHandler)
  {
    handler = messageHandler;
    bridge = responsHandler;
  }
  char logPath[MAX_FILENAME_SIZE];
  
private:
  /**
   * pointer to socket server - to get client connection */
  UServerPort* portServer;
  
  UDataItem * items[MAX_MESSAGE_COUNT];
  int itemsCnt = 0;
  int findDataItem(const char * item);
  UBridge * bridge;
  UHandler * handler;
  UTime tStart;
};

#endif
