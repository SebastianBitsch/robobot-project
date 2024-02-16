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

#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include "uregbot.h"
#include "ujoy.h"
#include "uhandler.h"
#include "udataitem.h"
#include "uoled.h"

//#define CLIENT_IS_BRIDGE -2


// typedef enum DI_DATA_ITEMS {
//   DI_HBT, // heartbeat
//   DI_JOY
// } DI_DATA_ITEMS;


UHandler::UHandler(UTeensy* robot, UData* data, UJoy * joystick, UOled * oledDisplay)
{
  bot = robot;
  items = data;
  joy = joystick;
  oled = oledDisplay;
  // no need to run thread (if robot is on-line) - maybe later
  // start();
}

UHandler::~UHandler()
{
  printf("Handler destroied\n");
}


// void UHandler::addDataItems()
// {
//   items->setItem(DI_HBT, "Heartbeat message", "hbt");
//   items->setItem(DI_JOY, "joystick status event", "joy");
// }

void UHandler::run()
{ // no need to run (if robot is on-line) - maybe later
  int loop = 0;
  sleep(1);  
  printf("Handler thread started\n");
  while (false and not th1stop)
  {
    usleep(100000);
    if (loop % 5 == 0)
    {
      const int MSL = 50;
      char s[MSL];
      snprintf(s, MSL, "%.3f 12.000 %.1f", loop*5.0/1000.0, joy->cputemp);
      items->updateItem("hbt", s, CLIENT_BRIDGE);
    }
    loop++;
  }
  printf("Handler thread stopped\n");
}


void UHandler::handleCommand(int client, char* msg, bool recursive)
{
  char * params = strchr(msg, '=');
  char * pplus  = strchr(msg, '+');
  char * pminus = strchr(msg, '-');
  char * pspace = strchr(msg, ' ');
  char keystr[MAX_ITEM_KEY_LENGTH + 1];
  char * key = keystr;
  //   int toRegbot = false;
  bool isOK = false;
  // more threads could enter, so make sure to handle one at a time.
//   handlerLock.lock();
  
  // debug
//   if (client >= 0 and client <= CLIENT_CONSOLE)
//   {
//     printf("UHandler::handleCommand client=%d cmd=%s\n", client, msg);
//   }
  // debug end
  if (msg[0] == '#')
  { // all remarks are handled as one message type
    isOK = true;
    strncpy(keystr, "#", MAX_ITEM_KEY_LENGTH);
    key = keystr;
    params = &msg[1]; // with hash stripped
  }
  else if (isdigit(msg[0]) or msg[0] == '%')
  { // log data from robot ram starts with a number (timestamp)
    // or a '%' as is a comment in matlab
    isOK = true;
    strncpy(keystr, "logdata", MAX_ITEM_KEY_LENGTH);
    key = keystr;
    params = &msg[0];
  }
  else if ((params != NULL and (params - msg) < MAX_ITEM_KEY_LENGTH) or
           (pspace != NULL and (pspace - msg) < MAX_ITEM_KEY_LENGTH) or
           (pplus  != NULL and (pplus  - msg) < MAX_ITEM_KEY_LENGTH) or
           (pminus != NULL and (pminus - msg) < MAX_ITEM_KEY_LENGTH)
          )
  { // a robot command with an equal sign, plus or minus - must be in the beginning
    // Find the first of these special separators
    if (params == NULL)
      params = &msg[MAX_ITEM_KEY_LENGTH];
    if (pspace != NULL and pspace < params)
      params = pspace;
    if (pplus != NULL and pplus < params)
      params = pplus;
    if (pminus != NULL and pminus < params)
      params = pminus;
    // move to first after separator
    params++;
    // find key length
    int n = params - msg; // keep the '=+-' as part of key
    if (msg[n-1] == ' ')
      // don't include space in key
      n--;
    if (n > MAX_ITEM_KEY_LENGTH)
      n = MAX_ITEM_KEY_LENGTH;
    // copy to new string
    strncpy(keystr, msg, n);
    keystr[n] = '\0';
    key = keystr;
    if (params == NULL)
    {
      params = &keystr[n];
      // debug
      // printf("UHandler::handleCommand: (1) key=%s, param=''\n", key);
    }
  }
  else
  { // key only and no separator character found
    strncpy(keystr, msg, MAX_ITEM_KEY_LENGTH);
    keystr[MAX_ITEM_KEY_LENGTH] = '\0';
    for (int i = strlen(keystr) - 1; i > 0; i--)
    { // remove potential \r\n - an trailing space
      if (keystr[i] <= ' ')
        keystr[i] = '\0';
      else
        break;
    }
    key = keystr;
    // make param an empty string
    params = &keystr[MAX_ITEM_KEY_LENGTH];
    // debug
//     printf("UHandler::handleCommand: (2) key=%s, param=''\n", key);
  }
  if (not isOK)
  {
    isOK = isalpha(key[0]) or key[0] == '<';
    if (not isOK)
      printf("UHandler::handleCommand: key failed: %s\n", key);
  }
  if (isOK and msg[0] != '#' and strlen(key) > 0)
    for (int i = 1; i < (int)strlen(key) and isOK; i++)
    { // key items are allowed to include '=', '+' ot '-' and alfanumeric characters
      isOK = isalnum(key[i]) or strchr("=+-", key[i]) != NULL;
    }
  if (isOK)
  { // valid key
    if (key != NULL)
    {
//       if (client >= 0 and isdigit(key[1]) and (key[0] == 'u' or key[0] == 'v'))
//       { // one of the major get data messages
//         toRegbot = true;
//       }
//       else if (client != CLIENT_ROBOT and (
//               strcmp(key, "start") == 0 or
//               strcmp(key, "stop") == 0
//               ))
//       { // other "old style" regbot messages
//         toRegbot = true;
//       }
//       else 
      { // assumed normal message
        if (false and client == 20)
          printf("UHandler::handleCommand: updating (client:%d, key: %s, params:%s)\r\n", client, key, params);
        // trim parameter list - also for \n\r\t etc.
        while (*params <= ' ' and *params != '\0')
          params++;
        for (int i = strlen(params) - 1; i > 0; i--)
        {
          if (params[i] <= ' ')
            params[i] = '\0';
          else
            break;
        }
        bool isLocked = false;
//         isLocked = handlerLock.try_lock();
        if (not recursive) // not isLocked and (client >=0 and client < CLIENT_CONSOLE))
        { // do not ignore something from "real" clients, especially start, stop and mission lines
          handlerLock.lock();
          isLocked = true;
        }
        if (isLocked or recursive) // (client < 0 or client == CLIENT_CONSOLE))
        {
          if (false and client == 20)
            printf("UHandler::handleCommand: updating locked=%d (client:%d, key: %s, params:%s)\r\n", isLocked, client, key, params);
          items->updateItem(key, params, client);
          if (isLocked)
            handlerLock.unlock();
          if (false and client == 20)
            printf("UHandler::handleCommand: updating unlocked=%d (client:%d, key: %s, params:%s)\r\n", isLocked, client, key, params);
        }
        else
        { // lock failed - may be dangerous
            printf("UHandler::handleCommand: failed to lock for client %d message '%s %s'\n", client, key, params);
        }
      }
//       if (client >= 0 and toRegbot)
//       { // not handled by bridge - send to robot
//         if (toRegbot == 2)
//         { // send params only - for all non-standard commands
//           bot->send(params,"",20);
//           printf("UHandler::handleCommand: sending params only to robot '%s'\r\n", params);
//         }
//         else
//           // send key and params
//           bot->send(key, params, 20);
//       }
    }
  }
  else
  {
    UTime t;
    t.now();
    printf("UHandler::handleCommand: %ld.%03ld ignored non-legal key (%s) in msg:(%s)\r\n", t.getSec(), t.getMilisec(), key, msg);
  }
}


