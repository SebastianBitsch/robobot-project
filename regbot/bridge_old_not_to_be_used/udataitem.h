/*
 * Handling of data items - i.e. messages
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

#ifndef UDATAITEM_H
#define UDATAITEM_H

#include <mutex>

#include "userverport.h"

#define MAX_PRIORITY 6
#define MAX_ITEM_KEY_LENGTH 8
// extra client is command line
#define EXTRA_CLIENTS       1
#define MAX_CLIENTS         (MAX_SOCKET_CLIENTS + EXTRA_CLIENTS)

class ULogFile;
class UBridge;
class UHandler;
class UTeensy;

class UDataItem
{
public:
  // descriptive name for this data item
  std::string itemName;
  // keyword (tag) for this message,
  // and also make space for string terminator  
  char itemKey[MAX_ITEM_KEY_LENGTH + 1];
  // value string
  std::string itemParams;
  // source update time for item
  UTime itemTime;
  // has data ever been updated
  bool itemValid;
  // Relay message to robot
//   bool destination;
  // true for one command only 'robot'
//   bool paramsOnly;
  // is this message sequence oriented, so that subscriber will get
  // a relay of all messages - as log list or mission lines
  bool dataSequence;
  /**
   * simpel update count */
  int updateCnt;
  /**
   * Update interval, time since last update (in seconds) */
  float updateInterval;
  /**
   * number of subscribe message from regbot 
   *  sub s p m    Subscribe s=1, unsubscribe s=0, p=priority 0 (high) .. 4(low), "
   *               "m=msg:0=hbt, 1=pose,2=IR,3=edge,4=acc,5=gyro,6=current,7=vel 
   * */
  int subscribeableFromRegbot = -1;
  
  
protected:
  // priority for this data for each client
  int priority[MAX_CLIENTS];
  // index to next client that has not-null priority
  int priorityNext[MAX_CLIENTS];
  // last time this item was send to this client
  UTime itemSend[MAX_CLIENTS];
  /**
   * desired interval time in seconds */
  float priorityTime[MAX_PRIORITY + 1];
  /**
   * pointer so socket server */
  UServerPort * serv;
  // avoid sending and updating at the same time
  std::mutex lock;
  // Relay to client default priority
  int clientDefPriority;
  // logfile
  ULogFile * logfile;
  // pointer to log-path stored elsewhere
  const char * logPath;
  // handler if an update involves handling by this bridge itself
  UBridge * respond;
  // responce number from the respond class.
  // this is zero or positive if there is a responce needed, else -1
  int respondNumber;
  // source of last update
  int source;
  
public:
  /** constructor */
  UDataItem(UServerPort * servPtr, const char * createdLogPath, UBridge * responceHandler);
  /** destructor */
  ~UDataItem();
  /**
   * Setting new source data 
   * \param newData is the data string
   * \param client is the sourse of the message
   */
  void updateItem(const char * key, char * params, int client);
  /**
   * Set new priority from this client
   * \param client is client number from server
   * \param priority is new priority - 0 is no priority
   * */
  void setPriority(int client, int newPriority, UTeensy * rob);
  /**
   * Get new priority from this client
   * \param client is client number from server
   * */
  int getPriority(int client)
  {
    return priority[client];
  }
  /**
   * Check time to send this message */
  void tick(UTime now);
  /**
   * Set item relay priorities */
//   void setRelayPriority(int toClientPri)
//   {
// //     destination = toRobot;
//     clientDefPriority = toClientPri;
//     for (int i = 0; i < MAX_CLIENTS; i++)
//       setPriority(i, toClientPri);
//   };
  /**
   * Send meta info back to client */
  void publishMeta(UHandler * handler, int client);
  
  /**
   * Get data item status */
  
protected:
  /**
   * Make link from relevant client to next
   * */
  void setPriorityList();
  /**
   * Send message to client
   * */
  void sendTo(int client);
  /**
   * Send meta information of data item */
  void sendMetaTo(int client);
  
  private:
  /** send item status to this client
   * \param client is index to client to get the status */
  void sendStatus(int client);
};

#endif
