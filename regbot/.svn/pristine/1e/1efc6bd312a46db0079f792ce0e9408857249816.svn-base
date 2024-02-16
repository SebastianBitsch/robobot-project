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

#include <mutex>
#include <string.h>

#include "userverport.h"
#include "userverclient.h"
#include "udataitem.h"
#include "ulogfile.h"
#include "ubridge.h"
#include "uhandler.h"
#include "uregbot.h"
#include "uregbot.h"

UDataItem::UDataItem(UServerPort * servPtr, const char * createdLogPath, UBridge * responceHandler)
{
  itemValid = false;
  updateCnt = 0;
  serv = servPtr;
  respond = responceHandler;
  itemKey[0] = '\0';
  updateInterval = 0.0; // in seconds
  itemName = "(no description)";
//   paramsOnly = false;
//   destination = false;
  dataSequence = false;
  logfile = NULL;
  logPath = createdLogPath;
  for (int i = 0; i < MAX_CLIENTS; i++)
  {
    priority[i] = 0;
    priorityNext[i] = 0;
  }
  itemTime.now();
  // set static priority timing (0 not used)
  priorityTime[0] = 0.002;
  for (int i = 1; i <= MAX_PRIORITY; i++)
  { // increase timing with factor 5: 0.01, 0.05, 0.25, 1.25, 6.25 seconds 
    priorityTime[i] = priorityTime[i-1] * 5;
  }
}
  
UDataItem::~UDataItem()
{
  if (logfile != NULL)
    logfile->closeLog();
//   if (subscribeableFromRegbot >= 0 and respond->getRegbot()->usbPortOpen)
//   {
//     const int MSL = 30;
//     char s[MSL];
//     snprintf(s, MSL, "sub 0 0 %d\n", subscribeableFromRegbot);
//     respond->getRegbot()->send(s, 100);
//     // debug
//     printf("### send a %s", s);
//   }
}

  
/**
  * Setting new source data */
void UDataItem::updateItem(const char * key, char * params, int client)
{
//   int dest = false;
  bool handled = false;
  //
  if (key == NULL)
  { // not a legal message
    return;
  }
  if (itemKey[0] == '\0')
  { // new message - set key
    strncpy(itemKey, key, MAX_ITEM_KEY_LENGTH);
    if (respond != NULL)
    {
      respondNumber = respond->getResponceNumber(key, client, &dataSequence);
//       printf("UDataItem::updateItem: Requesting respond number for '%s' - got %d for client %d\n", key, respondNumber, client);
    }
    else
      respondNumber = -1;
    if (client >= 0)
    { // may be data that could be requested from robot
      if (strcmp("hbt", key) == 0)
        subscribeableFromRegbot = 0; // heartbeat (battery voltage etc)
      else if (strcmp("pse", key) == 0)
        subscribeableFromRegbot = 1; // pose
      else if (strcmp("irc", key) == 0)
        subscribeableFromRegbot = 2; // IR sensor
      else if (strcmp("lip", key) == 0)
        subscribeableFromRegbot = 3; // line sensor
      else if (strcmp("acw", key) == 0)
        subscribeableFromRegbot = 4; // accelerometer
      else if (strcmp("gyw", key) == 0)
        subscribeableFromRegbot = 5; // gyro 
      else if (strcmp("mca", key) == 0)
        subscribeableFromRegbot = 6; // motor current
      else if (strcmp("wve", key) == 0)
        subscribeableFromRegbot = 7; // velocity
      else if (strcmp("mis", key) == 0)
        subscribeableFromRegbot = 8; // velocity
      if (subscribeableFromRegbot >= 0)
        printf("# %s is subscribeable from Regbot, as msg=%d\n", key, subscribeableFromRegbot);
    }
  }
  // check forst parameter for subcommand
//   if (params == NULL or strlen(params) == 0)
//   { // no params, so just request for data - send it right away
//     printf("UDataItem::updateItem: no params, reply to client %d\r\n", client);
//     lock.lock();
//     sendTo(client);
//     lock.unlock();
//     handled = true;
//   }
//   else
  if (params != NULL and strlen(params) > 0)
  { // there is more - check for management messages
    if (not isdigit(params[0]))
    { // this may be a management message
      if (strncmp(params, "subscribe ", 10) == 0)
      { // subscribe message get priority
        char * p1 = &params[10];
        int p = strtol(p1, NULL, 10);
        setPriority(client, p, respond->getRegbot());
        handled = true;
      }
      else if (strncmp(params, "status", 6) == 0)
      { // send item status to requester
        lock.lock();
        sendStatus(client);
        lock.unlock();
        handled = true;
      }
      else if (strncmp(params, "name ", 5) == 0)
      { // set item name
        char * p1 = &params[5];
        // copy to name/description
        itemName = p1;
        handled = true;
      }
      else if (strncmp(params, "set ", 4) == 0)
      { // set item name
        char * p1 = &params[4];
        
        dataSequence = strtol(p1, &p1, 10);
        // the rest is item name/explanation
        // itemName is a c++ 'string', so make a copy
        itemName = p1;
        handled = true;
      }
      else if (strncmp(params, "meta", 3) == 0 and client >= 0)
      { // not for robot or joy
        lock.lock();
        sendMetaTo(client);
        lock.unlock();
        handled = true;
      }
      else if (strncmp(params, "get", 3) == 0 and client >= 0)
      { // not for robot or joy
        lock.lock();
        sendTo(client);
        lock.unlock();
        handled = true;
      }
      else if (strncmp(params, "logopen", 7) == 0 and client >= 0)
      { // not for robot or joy
        lock.lock();
        if (logfile == NULL)
        {
          logfile = new ULogFile(key, logPath);
          logfile->openLog(true);
          if (logfile->isOpen())
          { // add description as first line in log (with matlab comment)
            fprintf(logfile->getF(), "%% logfile for item %s\n", itemKey);
            fprintf(logfile->getF(), "%% %s\n", itemName.c_str());
            printf("UDataItem::updateItem - opened logfile %s\n", logfile->getLogFileName());
          }
        }
        lock.unlock();
        handled = true;
      }
      else if (strncmp(params, "logclose", 8) == 0 and client >= 0)
      { // not for robot or joy
        lock.lock();
        if (logfile != NULL)
        {
          logfile->closeLog();
          printf("UDataItem::updateItem - closed logfile %s\n", logfile->getLogFileName());
        }
        lock.unlock();
        handled = true;
      }
      else if (((params[0] == 'h' and params[1] <= ' ') or (strncmp(params, "help", 4) == 0)) and client >= 0)
      { // not for robot or joy
        serv->sendString("# help for special second parameter:\r\n", client);;
        serv->sendString("#   get          Gets the value of the tata item\r\n", client);;
        serv->sendString("#   meta         Gets 'key meta r vs s p description': r=responder, vs: 0=val 1=seq, s=source, p=priority\r\n", client);;
        serv->sendString("#   subscribe p  Set subscription priority 0=none, 1=fast (10ms), 5 = slow (6sec), 6=all updates \r\n", client);;
//         serv->sendString("   set d s  Sets meta, d=destination 1=robot, 2=bridge,3=display,+64=params only, s=sequence 0=time value, 1=command etc\r\n", client);;
        serv->sendString("#   status       Sends status 'key status c T n p p p p ...' c: update count, T: since last (sec), n=clients slots, p client priority\r\n", client);;
        serv->sendString("#   name xxx     Sets name or description for data item\r\n", client);
        serv->sendString("#   logopen      Opens a (new) logfile and log all updates with timestamp (key.txt)\r\n", client);
        serv->sendString("#   logclose     Closes logfile (if open)\r\n", client);
        serv->sendString("#   h            This help\r\n", client);
        if (strncmp(params, "help", 4) != 0)
          handled = true;
      }
    }
  }
  if (not handled)
  { // normal update message
//     dest = destination;
//     if (paramsOnly)
//     {
//       dest += 64;
//       printf("UDataItem::updateItem dest=%d\n", dest);
//     }
//    printf("UDataItem::updateItem: not handled debug 1\n");
    lock.lock();
//    printf("UDataItem::updateItem: not handled debug 2\n");
    UTime t;
    t.now();
    float dt = t - itemTime;
    // average update time a bit - or maybe not
    updateInterval = dt; // updateInterval * 3/4 + dt/4;
    itemTime = t;
    source = client;
    itemParams = params;
    itemValid = true;
    if (respondNumber >= 0)
      respond->responce(respondNumber, this, client);
    if (logfile != NULL)
    {
      if (logfile->isOpen())
        logfile->toLog(t, params);
    }
    lock.unlock();
    // run a tick to get as
    // fresh data as possible
    // to clients
    updateCnt++;
    tick(itemTime);
  }
}
  
void UDataItem::sendStatus(int client)
{
  const int MSL = 1000;
  char s[MSL];
  char * p1 = s;
  int n = 0;
  snprintf(p1, MSL - n, "%s status %d %.3f %d",  itemKey, updateCnt, updateInterval, MAX_CLIENTS);
  for (int i = 0; i < MAX_CLIENTS; i++)
  {
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, MSL - n, " %d", priority[i]);
  }
  n += strlen(p1);
  p1 = &s[n];
  snprintf(p1, MSL - n, "\r\n");
  serv->sendString(s, client);
}

/**
  * Set new subscribe priority from this client
  * \param client is client number from server
  * \param priority is new priority - 0 is no priority
  * */
void UDataItem::setPriority(int client, int newPriority, UTeensy * rob)
{
  int oldPriority = 0;
  if (client >= 0 and client < MAX_CLIENTS)
  {
    oldPriority = priority[client];
    if (newPriority < 0 or newPriority > MAX_PRIORITY)
      priority[client] = 0;
    else
      priority[client] = newPriority;
  }
  // set priority list links
  setPriorityList();
  //
  // manage subscribale data source from Regbot
  if (subscribeableFromRegbot >= 0)
  { // sub s p m    Subscribe s=1, unsubscribe s=0, p=priority 0 (high) .. 4(low), "
    //              "m=msg:0=hbt, 1=pose,2=IR,3=edge,4=acc,5=gyro,6=current,7=vel 
    printf("UDataItem::setPriority: for %s, sub msg=%d, client=%d p=%d\n", itemName.c_str(), subscribeableFromRegbot, client, newPriority);
    if (rob != NULL)
    { // robot link available
      int highestPriority = 6; // priority 6 is just when available ( assumed to be polled data)
      const int MSL = 30;
      char s[MSL];
      for (int i = 0; i < MAX_CLIENTS; i++)
      {
        if (priority[i] != 0 and priority[i] < highestPriority)
          highestPriority = priority[i];
      }
      if (highestPriority < 6)
      { // change subscription
        if (highestPriority > 4)
          highestPriority = 4;
        snprintf(s, MSL, "sub 1 %d %d\n", highestPriority, subscribeableFromRegbot);
        rob->send(s, 50);        
      }
      else if (subscribeableFromRegbot > 0 and oldPriority > 0)
      { // remove subscription (except for heartbeat)
        snprintf(s, MSL, "sub 0 0 %d\n", subscribeableFromRegbot);
        rob->send(s, 50);        
      }
    }
  }
}

/**
  * Check time to send this message */
void UDataItem::tick(UTime now)
{
  if (priority[0] or priorityNext[0])
  { // there may be something to send
    lock.lock();
    float dti = now - itemTime; // age of data
    float dt = now - itemSend[0]; // time since last send this item to this client
    if ((priority[0] and dt > priorityTime[priority[0]]) or (priority[0] == MAX_PRIORITY))
    { // client 0 is treated first
      sendTo(0);
    }
    int c = 0;
    while (priorityNext[c])
    { // goto next client
      c = priorityNext[c];
      dt = now - itemSend[c];
      if ((dt > priorityTime[priority[c]] and dt > dti) or priority[c] == MAX_PRIORITY)
        sendTo(c);
    }
    lock.unlock();
  }
}

/**
  * Make link from relevant client to next
  * */
void UDataItem::setPriorityList()
{
  int last = 0;     
  for (int i = MAX_CLIENTS-1; i >=0; i--)
  {
    priorityNext[i] = last;
    if (priority[i])
      last = i;
  }
}
/**
  * Send message to client
  * */
void UDataItem::sendTo(int client)
{
  if (itemValid)
  {
    bool isOK = false;
    const int MSL = 3000;
    char s[MSL];
    snprintf(s, MSL, "%s %s\r\n", itemKey, itemParams.c_str());
    isOK = serv->sendString(s, client);
//     printf("UDataItem::sendTo: to client %d send %s %s\n", client, itemKey, itemParams.c_str());
    if (isOK)
    { // note send time
      itemSend[client].now();
    }
    else
    { // client lost - remove priotity
      priority[client] = 0;
      // and relist
      setPriorityList();
    }
  }
  else
  {
    printf("UDataItem::sendTo: item %s not valid\n", itemKey);
    const int MSL = 60;
    char s[MSL];
    snprintf(s, MSL, "# key=%s is not valid (not found)\r\n", itemKey);
    serv->sendString(s, client);
  }
}


void UDataItem::sendMetaTo(int client)
{
  bool isOK = false;
  const int MSL = 3000;
  char s[MSL];
  snprintf(s, MSL, "# %s meta %d %d %d %d %s\r\n", itemKey, respondNumber, dataSequence, source, priority[client], itemName.c_str());
  isOK = serv->sendString(s, client);
//   printf("UDataItem::sendTo: to client %d send a %s\r\n", client, itemKey);
  if (not isOK)
  { // client lost - remove priotity
    priority[client] = 0;
    // and relist
    setPriorityList();
  }
}

void UDataItem::publishMeta(UHandler * handler, int client)
{
  const int MSL = 3000;
  char s[MSL];
//   snprintf(s, MSL, "# bridge data %s %s\r\n", itemKey, itemName.c_str());
  snprintf(s, MSL, "# data item '%s'     %s\r\n", itemKey, itemName.c_str());
//   handler->handleCommand(CLIENT_BRIDGE, s);
  serv->sendString(s, client);
}
