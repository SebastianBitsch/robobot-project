/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
 *   has an IMU and a dual motor controller with current feedback.
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef WIFI_8266_H
#define WIFI_8266_H

#include <string.h>
#include <stdlib.h>
#include "WProgram.h"

//////////////////////////////////////

class UWifi8266;
extern UWifi8266 wifi;

/////////////////////////////////////

class UWifi8266
{
public:
  void sendStatusWiFiHuman();

  void sendStatusWiFi();

  void sendStatusWiFiClients();

  void decodeWifi(char * buf);

  int8_t wait4Reply(uint32_t timeout_ms);

  bool serialSetup();
  /** send string to wifi channel
   * \param link is link to send to [0..4]
   * \param str is string to send 
   * \param cnt is number of characters to send
   * \param addCRLF if true, then \\r + \\n is added 
   * \returns true if not busy */
  bool wifiSend(int link, const char * msg, int cnt, bool addCRNL = false);
  /**
   * Handle incoming data from serial line (8266).
   * \param n is new char from line
   * \param cmd (out) is pointer to data from serial line - received from a client.
   * \param channel (out) client channel [0..4] is returned here.
   * \returns true if a line is received (terminated with \n. */
  bool receivedCharFromSer(uint8_t n, char ** cmd, int * channel);

public:
  static const int TX_BUF_SIZE = 400;
  char serTxBuf[TX_BUF_SIZE]; // wifi tx buffer (one message only)
  //bool waitForSendOK = false;
  uint32_t waitForSendOKtime;
  int8_t setup = -1;
  static const int WIFI_MAX_CLIENTS = 5; // 
  typedef enum
  {
    WIFI_NOT = 0, // not alive
    WIFI_MUST,    // alive do not send status
    WIFI_NO_HASH, // alive send all but no debug starting with #
    WIFI_ALL      // alive send all (as to USB)
  } WIFI_MGS_TYPE;
  WIFI_MGS_TYPE clientActive[WIFI_MAX_CLIENTS] = {WIFI_NOT,WIFI_NOT,WIFI_NOT,WIFI_NOT,WIFI_NOT};
  uint32_t clientAlive[WIFI_MAX_CLIENTS] = {0,0,0,0,0};

  typedef enum 
  {
    WFS_NONE = 0,      // not started
    WFS_NO_CONNECTION, // from message
    WFS_CONNECTED,     // claimed by message
    WFS_GOT_IP,        // claimed by message
    WFS_ALL_OK         // got real IP number from 8266 (assumes all is OK)
  } WifiStatus;
  typedef enum 
  {
    WFI_FINISHED = 0,  // not expecting a reply
    WFI_NO_REPLY_YET,  // expecting a reply
    WFI_OK,            // got an OK reply
    WFI_ERR,
    WFI_BUSY,
    WFI_DATA,
    WFI_LINK_NOT_VALID,
    WFI_MESSAGE
  } WifiReply;
  typedef enum 
  {
    WFD_NONE = 0,  // not in sending mode
    WFD_SEND_REQ,  // waiting for OK to transfer data
    WFD_SENDING,   // transferring data and sending
    WFD_SEND_OK,    // all data send OK
    WFD_SEND_FAILED
  } WifiSending;
  WifiStatus status = WFS_NONE;
  WifiReply replyType = WFI_FINISHED;
  WifiSending sendingState = WFD_NONE;
  // debug
  static const int RX_BUF_SIZE = 110;
  char serRxBuf[RX_BUF_SIZE]; // wifi connection
  int serRxBufCnt = 0;
  
  /// save wifi settings to ee prom
  void eePromSaveWifi();
  /// load wifi settings from ee-prom
  void eePromLoadWifi();
  
protected:
  uint8_t replyState = 0;
  uint32_t wifiWait = 0;
  uint32_t wifiBusy = 0;
  uint8_t wifiFailCnt = 0;
  uint8_t wifiIP[4]; // IP number from wifi
  char wifiMAC[18] = "00:00:00:00:00:00"; // STATIC MAC as string
  uint16_t portNumber = 24041;
//  bool wifiWaitForSendOK = false;
  uint32_t wifiMsgGood = 0, wifiMsgLost = 0;
  //float wifiLoss = 0.0;
//  uint32_t wifiWaitForSendOKtime;
  static const int MAX_SSID_SIZE = 16;
//  char wifiSSID[MAX_SSID_SIZE] = "WaveLAN IAU";
  char wifiSSID[MAX_SSID_SIZE] = "device";
  char wifiPW[MAX_SSID_SIZE] = "";
  
  ////////////////
private:
  uint32_t wifiTiming = 0;
  uint32_t wifiDelayMax = 0;
  uint32_t wifiDelayMax2 = 0;
  
};


#endif