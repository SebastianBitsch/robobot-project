/*
 * Control of data transfer
 * from socket client 
 * Services data from regbot and joystick - and possibly oled display
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
#include <streambuf>

#include "udata.h"
#include "udataitem.h"

#define REV_ID "$Id: command.cpp 791 2018-01-01 19:44:32Z jcan $" 




UData::UData(UServerPort * port)
{
  portServer = port;
  itemsCnt = 0;
  for (int i = 0; i < MAX_MESSAGE_COUNT; i++)
  {
    items[i] = NULL;
  }
  // create directory for logfiles
  tStart.now();
  strncpy(logPath, "log_", MAX_FILENAME_SIZE);
  tStart.getForFilename(&logPath[4]);
  // remove old empty dirs
  system ("rmdir log_20* 2>/dev/null\n");
  // make a new dir for this session
  const int MSL = MAX_FILENAME_SIZE + 20;
  char s[MSL];
  snprintf(s, MSL, "mkdir -p %s\n", logPath);
  system(s);
}

UData::~UData()
{
  for (int i = 0; i < MAX_MESSAGE_COUNT; i++)
  {
    if (items[i] != NULL)
    {
      delete items[i];
      items[i] = NULL;
    }
  }
}


int UData::findDataItem(const char * item)
{
  int idx = -1;
  for (int i = 0; i < itemsCnt; i++)
  {
    UDataItem * d = items[i];
    if (strlen(item) == strlen(d->itemKey))
    { // same key length
      if (strcmp(d->itemKey, item) == 0)
      {
        idx = i;
        break;
      }
    }
  }
  return idx;
}


void UData::updateItem(const char * key, char * params, int client)
{
  int itemIndex;
//   int forRobot = false;
  itemIndex = findDataItem(key);
  UDataItem * d = NULL;
//   printf("UData::updateItem: data item %d/%d\n", itemIndex, itemsCnt);
  if (itemIndex < 0)
  {
    if (itemsCnt < MAX_MESSAGE_COUNT)
    { // create a new data subject
      d = new UDataItem(portServer, logPath, bridge);
//       if (strcmp(key, "robot") == 0)
//       { // debug
//         printf("UData::updateItem, new robot data item (key=%s, params=%s)\n", key, params);
//       }
      items[itemsCnt++] = d;
    }
    else
      printf("UData::updateItem: no more space for new data items\n");
  }
  else
    d = items[itemIndex];
  if (d != NULL)
  {
    d->updateItem(key, params, client);
  }
//   return forRobot;
}

// void UData::setPriority(int client, int newPriority, int itemIndex)
// {
//   if (itemIndex >= 0 and itemIndex < MAX_MESSAGE_COUNT and 
//     client >= 0 and client < MAX_SOCKET_CLIENTS_SERVED)
//   {
//     if (items[itemIndex] == NULL)
//     {
//       items[itemIndex] = new UDataItem(portServer);
//       if (itemIndex >= itemsCnt)
//         itemsCnt = itemIndex + 1;
//       const int MSL = 50;
//       char s[MSL];
//       snprintf(s, MSL, "# no %s data yet\n", items[itemIndex]->itemName.c_str());
//       items[itemIndex]->updateItem(s);
//     }
//     items[itemIndex]->setPriority(client, newPriority);
//   }
// }

void UData::setItem(int index, std::string itemName, const char * itemKey)
{
  if (index >= 0 and index < MAX_MESSAGE_COUNT)
  {
    items[index]->itemName = itemName;
    strncpy(items[index]->itemKey, itemKey, MAX_ITEM_KEY_LENGTH);
    items[index]->itemKey[MAX_ITEM_KEY_LENGTH] = '\0';
  }
}


void UData::printStatus()
{
  printf("#recorded %d items:\n", this->itemsCnt);
  for (int i = 0; i< itemsCnt; i++)
  {
    UDataItem * d = items[i];
    printf("   item %d is %6s, upds=%3d, dt=%.3fs, valid=%d, time=%.3fs '%s'\r\n", 
           i, d->itemKey, d->updateCnt, d->updateInterval, d->itemValid,  d->itemTime - tStart, d->itemParams.c_str());
    printf("                           %s\r\n", d->itemName.c_str());
  }
}

int UData::publishItemMeta(int item, int client)
{
  int reply = -1;
  if (item >= 0 and item < itemsCnt)
  {
    reply = item;
    UDataItem * d = items[item];
    d->publishMeta(handler, client);
  }
  return reply;
}
