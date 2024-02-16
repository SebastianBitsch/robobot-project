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


#include "wifi8266.h"
#include "main.h"
#include "command.h"
#include "eeconfig.h"

UWifi8266 wifi;

////////////////////////////////////////////////////////////

void UWifi8266::sendStatusWiFiHuman()
{
  const int MSL = 120;
  char s[MSL + 4];
  int n = 0;
  char * p1;
  snprintf(s, MSL, "# WiFi (setup=%d) " , setup);
  n = strlen(s);
  p1 = &s[n];
  switch (replyType)
  {
    case WFI_FINISHED: strncpy(p1, "FINISHED, ", MSL - n); break;
    case WFI_NO_REPLY_YET: strncpy(p1, "NO REPLY YET, ", MSL - n); break;
    case WFI_OK: strncpy(p1, "OK, ", MSL - n); break;
    case WFI_ERR: strncpy(p1, "ERR, ", MSL - n); break;
    case WFI_BUSY: strncpy(p1, "BUSY, ", MSL - n); break;
    case WFI_DATA: strncpy(p1, "DATA, ", MSL - n); break;
    case WFI_LINK_NOT_VALID: strncpy(p1, "NO LINK, ", MSL - n); break;
    case WFI_MESSAGE: strncpy(p1, "MESSAGE, ", MSL - n); break;
  }
  n += strlen(p1);
  p1 = &s[n];
  switch (status)
  {
    case WFS_NONE: strncpy(p1, "not started", MSL - n); break;
    case WFS_NO_CONNECTION: strncpy(p1, "not connected", MSL - n); break;
    case WFS_CONNECTED: strncpy(p1, "connected (no IP)", MSL - n); break;
    case WFS_GOT_IP: strncpy(p1, "got IP", MSL - n); break;
    case WFS_ALL_OK: strncpy(p1, "all OK", MSL - n); break;
  }
  n += strlen(p1);
  p1 = &s[n];
  if (status >= WFS_GOT_IP)
  {
    snprintf(p1, MSL - n, ", IP=%d.%d.%d.%d ", wifiIP[0], wifiIP[1], wifiIP[2], wifiIP[3]);
    n += strlen(p1);
    p1 = &s[n];
  }
  strncpy(p1, "\r\n", MSL - n);
  usb_send_str(s);
}

/**
 * Send current status for WiFi - client readable */
void UWifi8266::sendStatusWiFi()
{
  const int MSL = 120;
  char s[MSL + 4];
  snprintf(s, MSL, "wfi %d %d %d  %d %d %d %d  %s %d %s\r\n" , setup, status, replyType, 
           wifiIP[0], wifiIP[1], wifiIP[2], wifiIP[3], 
           wifiMAC,
           portNumber, wifiSSID);
  usb_send_str(s);
  if (localEcho)
    // also to terminal
    sendStatusWiFiHuman();
}

void UWifi8266::sendStatusWiFiClients()
{
  const int MSL = 120;
  char s[MSL + 4];
  uint32_t a[WIFI_MAX_CLIENTS];
  for (int i = 0; i < WIFI_MAX_CLIENTS; i++)
    if (clientActive[i])
      a[i] = hbTimerCnt - clientAlive[i];
    else
      a[i] = 0;
    snprintf(s, MSL, "wfc %d %lu  %d %lu  %d %lu  %d %lu  %d %lu  %lu %lu\r\n" , 
             clientActive[0], a[0],
             clientActive[1], a[1],
             clientActive[2], a[2],
             clientActive[3], a[3],
             clientActive[4], a[4], wifiMsgGood, wifiMsgLost);
    usb_send_str(s);
    wifiMsgLost = 0;
    wifiMsgGood = 0;
}

/**
 * Decode wifi message with connect parameters in format
 * "wifi 1 port SSID", where
 * \param 1 marks that connection shall be (re)initialized (1 = reinitialize, 0=not)
 * \param port is the port number to open
 * \param SSID is the wifi access point name
 * \param buf is the string holding all these parameters */
void UWifi8266::decodeWifi(char * buf)
{
  char * p1 = buf, *p2;
  int v;
  v = strtol(p1, &p1, 10);
  if (*p1 > '\0')
    setup = v;
  else
    usb_send_str("# wifi: missing parameters\r\n");
  v = strtol(p1, &p1, 10);
  if (*p1 > '\0' and v >1000)
  {
    if (v != portNumber)
      portNumber = v;
  }
  else
    usb_send_str("# wifi: port must be > 1000\r\n");
  // find start of SSID
  while (*p1 > 0 and *p1 != '"')
    p1++;
  if (*p1 == '"')
  { // end SSRID at new line
    p2 = ++p1;
    while (*p2 > '\0' and *p2 != '"')
      p2++;
    if (*p2 == '"')
    { // debug
      //       const int MSL = 75;
      //       char s[MSL];
      // debug end
      *p2++ = '\0';
      strncpy(wifiSSID, p1, MAX_SSID_SIZE);
      // now get password - if exist
      wifiPW[0] = '\0';
      // now 
      // debug
      //       snprintf(s, MSL, "#wifi got: p1=%s, p2=%s, of:%s:\r\n", p1, p2, buf);
      //       usb_send_str(s);
      // debug end
      p1 = p2;
      while (*p1 > 0 and *p1 != '"')
        p1++;
      if (*p1 == '"')
      { // end SSRID at new line
        p2 = ++p1;
        while (*p2 > 0 and *p2 != '"')
          p2++;
        if (*p2 == '"')
        { // remove quote and terminate
          // debug
          //           snprintf(s, MSL, "#wifi got: p1=%s, p2=%s, of:%s:\r\n", p1, p2, buf);
          //           usb_send_str(s);
          // debug end
          *p2 = '\0';
          strncpy(wifiPW, p1, MAX_SSID_SIZE);
        }    
        else
          usb_send_str("# wifi: failed to find end quote PW\r\n");
      }
      else
        usb_send_str("# wifi: failed to find quoted PW\r\n");
    }
    else
      usb_send_str("# wifi: failed to find end quote for SSID\r\n");
  }
  else
    usb_send_str("# wifi: failed to find quoted SSID\r\n");
  // debug
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "# wifi set (%s) - now SSID=%s, PW=%s, port=%d\r\n", buf, wifiSSID, wifiPW, portNumber);
  usb_send_str(s);
  // debug end
}

int8_t UWifi8266::wait4Reply(uint32_t timeout_ms)
{
  int8_t result = 0;
  const int MSL = 80;
  char s[MSL];
  switch (replyState)
  {
    case 1:
      // a commad is send, start timeout counter
      wifiWait = hbTimerCnt;
      // wait for reply
      replyState++;
      break;
    case 2:
      // waiting for reply
      if (replyType == WFI_OK)
      { // we got an OK, reset timer and proceed
        result = 1;
        if (true or localEcho)
        {
          snprintf(s, MSL, "#\n... took %lu ms (to OK)\r\n", hbTimerCnt - wifiWait);
          usb_send_str(s);
        }
        //         usb_send_str("# WFI OK\r\n");
        // sendStatusWiFi();
      }
      else if (replyType == WFI_ERR)
      { // retry right away
        result = -1;
        wifiFailCnt++;
        if (true or localEcho)
        {
          snprintf(s, MSL, "#\n... took %lu ms (to fail %d times)\r\n", hbTimerCnt - wifiWait, wifiFailCnt);
          usb_send_str(s);
        }
        usb_send_str("# WFI error\r\n");
        if (wifiFailCnt > 2)
        {
          usb_send_str("# WFI error - end of retry - give up\r\n");
          result = -2;
          setup = 0;
        }
        //sendStatusWiFiHuman();
      }
      else if (replyType == WFI_BUSY)
      { // retry af a short while
        usb_send_str("# - BUSY wait\r\n");
        if (localEcho)
        {
          snprintf(s, MSL, "#\n... took %lu ms (to busy)\r\n", hbTimerCnt - wifiWait);
          usb_send_str(s);
        }
        replyState++;
        wifiBusy = hbTimerCnt;
        usb_send_str("# WFI busy\r\n");
        //         sendStatusWiFiHuman();
      }
      else if (hbTimerCnt - wifiWait > timeout_ms)
      { // giving up
        status = WFS_NONE;
        result = -2;
        usb_send_str("# WFI timeout\r\n");
        sendStatusWiFiHuman();
      }
      break;
      case 3:
        // chip is busy, so wait a bit (1s) and then retry
        if (hbTimerCnt - wifiBusy > 1000)
        { // finished waiting - retry command
          result = -1;
          if (localEcho)
          {
            snprintf(s, MSL, "#\n... took %lu ms (busy end wait)\r\n", hbTimerCnt - wifiBusy);
            usb_send_str(s);
          }
        }
        else
          // wait a bit longer
          wifiBusy--;
        break;
      default:
        usb_send_str("# wifi (wait4reply) state error\n");
        replyState = 0;
        break;
  }
  if (result != 0)
  { // there is a conclusion - either retry or continue - or error
    replyState = 0;
    // and we are no longer waiting for a reply
    replyType = WFI_FINISHED;
  }
  return result;
}

/**
 * Initialize wifi and set up socket server.
 * Assumed to be called at every loop (until finished).
 * Result is set by the "receivedCharFromSer" function.
 */
bool UWifi8266::serialSetup()
{
  const int MSL = 65;
  char s[MSL];
  int8_t w;
  if (replyState == 0)
  {
    switch (setup)
    {
      case 0: // do not setup
        break;
      case 1:
        // Serial1.write("AT+RST\r\n");
        // wait for confirmation
        // wifiReplyState = 1;
        setup = 2;
        wifiFailCnt = 0;
        // debug
        snprintf(s, MSL, "# ---- wifi setup to '%s', pw='%s', port %d ----\r\n", 
                 wifiSSID, wifiPW, portNumber); 
        usb_send_str(s);
        // debug end
        break;
      case 2:
        Serial1.write("AT+CWMODE=3\r\n");
        // wait for confirmation
        replyType = WFI_NO_REPLY_YET;
        // debug
        //         usb_send_str("# Send MODE=3\r\n");
        // debug end
        break;
      case 3:
        snprintf(s, MSL, "AT+CWJAP=\"%s\",\"%s\"\r\n", wifiSSID, wifiPW);
        Serial1.write(s);
        // wait for confirmation
        replyType = WFI_NO_REPLY_YET;
        // debug
        //         snprintf(s, MSL, "# send: AT+CWJAP=\"%s\",\"%s\"\r\n", wifiSSID, wifiPW);
        //         usb_send_str(s);
        //         sendStatusWiFiHuman();
        // debug end
        break;
      case 4:
        // waiting for "got IP"
        //         if ((hbTimerCnt - wifiWait) > 8000)
        //         { // waited 8 sec for IP, give up
        //           wifiStatus = WFS_NONE;
        //           usb_send_str("# failed to connect to get IP\r\n");
        //           setup = 0;
        //           // debug
        // //           sendStatusWiFi();
        // //           sendStatusWiFiHuman();
        //           // debug end
        //         }        
        //         if (wifiStatus == WFS_GOT_IP)
      { // ready to read IP
        setup++;
      }
      break;
      case 5:
        // enable multiple connections (5)
        Serial1.write("AT+CIPMUX=1\r\n");
        // wait for confirmation
        replyType = WFI_NO_REPLY_YET;
        // debug
        //         usb_send_str("# Send CIPMUX=1 (multible connections)\r\n");
        // debug end
        break;
      case 6:
        // set as socket server
        snprintf(s, MSL, "AT+CIPSERVER=1,%d\r\n", portNumber);
        Serial1.write(s);
        // wait for confirmation
        replyType = WFI_NO_REPLY_YET;
        // debug
        //         usb_send_str("# Send CIPMUX=240XX (XX=robot ID)\r\n");
        // debug end
        break;
      case 7:
        // get IP
        Serial1.write("AT+CIFSR\r\n");
        // wait for confirmation
        replyType = WFI_NO_REPLY_YET;
        // debug
        //         usb_send_str("# Send AT+CIFSR get IP\r\n");
        // debuge end
        break;
      case 99:
        break;
      default:
        setup = 99;
        // debug
        //         sendStatusWiFi();
        //         sendStatusWiFiHuman();
        //wifiStatus = WFS_GOT_IP;
        break;
    }
    if (replyType == WFI_NO_REPLY_YET)
      // activate reply state machine
      replyState = 1;
  }
  else
  { // wait for reply (OK, error or otherwise)
    w = wait4Reply(10000);
    if (w == -2)
      setup = 0; // gave up
      else
        // redo, continue or no change
        setup += w;
  }
  return setup == 99;
}

/**
 * Got a new character from wifi channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
bool UWifi8266::receivedCharFromSer(uint8_t n, char ** cmd, int * channel)
{ // got another character from usb host (command)
  const int MSL = 80;
  char s[MSL];
  bool result = false;
  if (n >= '\n')
  { 
    if (serRxBufCnt > 0 or n >= ' ')
    { // ignore leading whitespace
      serRxBuf[serRxBufCnt] = n;
      if (serRxBufCnt < RX_BUF_SIZE - 1)
        serRxBuf[++serRxBufCnt] = '\0';
    }
  }
  //   if (localEcho == 1)
  //       // echo characters back to terminal
  //       Serial1.write(n);
  if (sendingState == WFD_SEND_REQ)
  { // we are waiting for 
    if (serRxBuf[0] == '>')
    {
      wifiTiming = hbTimerCnt;
      Serial1.write(serTxBuf);
      sendingState = WFD_SENDING;
      // debug
      uint32_t t = hbTimerCnt - wifiTiming;
      uint32_t t2 = hbTimerCnt - waitForSendOKtime;
      if (t2 > wifiDelayMax2 or t > wifiDelayMax)
      {
        if (t2 > wifiDelayMax2)
        {
          wifiDelayMax2 = t2;
          snprintf(s, MSL, "# wifi send 2 delay=%lu ms (write %lu ms)\r\n", t2, t);
        }
        else
        {
          wifiDelayMax = t;
          snprintf(s, MSL, "# wifi send 1 (delay=%lu ms) write %lu ms\r\n", t2, t);
        }
        usb_send_str(s);          
      }
//       else
//       {
//         snprintf(s, MSL, "# wifi send to Serial1 (delay=%lu ms) write %lu ms\r\n", t2, t);
//         usb_send_str(s);          
//       }
      // debug end
//       waitForSendOK = false;
      serRxBufCnt = 0;
    }
    // debug
    else
    { 
      char * p1 = strchr(serRxBuf, '>');
      if (p1 != NULL)
      {
        snprintf(s, MSL, "# wifi found '>' in a strange place '%s'\r\n", serRxBuf);
        usb_send_str(s);
      }
    }
    // debug end
  }
  if ((n == '\n') and serRxBufCnt > 1)
  { // zero terminate
    char * p1, *p2, *p3, *p4;
    int serChannel;
    const int MSL = 80;
    char s[MSL];
    serRxBuf[serRxBufCnt] = '\0';
    //
    //     if (localEcho)
    //     {
    //       usb_send_str(serRxBuf);
    //       usb_send_str("\r\n");
    //     }
    // usb_send_str("(1)\r\n");
    p1 = strstr(serRxBuf, "+IPD,"); // received data
    p2 = strstr(serRxBuf, "OK");    // an acknowledge, either OK or SEND OK
    p3 = strstr(serRxBuf, ",CONNECT");// probably 0,CONNECT
    p4 = strstr(serRxBuf, ",CLOSED"); // propably '0,CLOSED'
    // debug
    //     {
    //       const int MSL = 75;
    //       char s[MSL];
    //       snprintf(s, MSL, "# wifi got %d chars ¤%s¤\r\n", serRxBufCnt, serRxBuf);
    //       usb_send_str(s);
    //     }
    // debug end
    if (p1 != NULL)
    {
      //        usb_send_str(p1 + 5);
      //        usb_send_str("(2)\r\n");
      serChannel = strtol(p1 + 5, &p1, 10);
      if (serChannel >= 0  and p1 != NULL)
      {
        p1++;
        //         usb_send_str(p1);
        //         usb_send_str("(3)\r\n");
        int n = strtol(p1, &p1, 10);
        if (p1 != NULL and n > 0)
        { // there is valid data, return data
          p1++;
          result = true;
          *cmd = p1;
          *channel = serChannel;
          if (strncmp(p1, "alive", 5) == 0 or strncmp(p1, "<alive", 6) == 0)
          {  // just an alive signal - note
            clientAlive[serChannel] = hbTimerCnt;
            snprintf(s, MSL, "<alive last=\"%.5f\"/>\r\n", float(controlUsedTime[1]) / 100000.0);
            wifi.wifiSend(serChannel, s, strlen(s));
          }
//           else
//             parse_and_execute_command(p1, n, serChannel);
        }
      }
      replyType = WFI_DATA;
    }
    else if (p3 != NULL)
    {
      int v = strtol(serRxBuf, &p3, 10);
      if (*p3 == ',' and v >= 0 and v < WIFI_MAX_CLIENTS)
      {
        clientActive[v] = WIFI_MUST;
        clientAlive[v] = hbTimerCnt;
        // debug
        //         usb_send_str("#activated wifi client\r\n");
        // debug end
      }
      // debug
      else if (localEcho)
        usb_send_str("# CONNECT failed to activate client\r\n");
      // debug end
    }
    else if (p4 != NULL)
    {
      int v = strtol(serRxBuf, &p4, 10);
      if (*p4 == ',' and v >= 0 and v < WIFI_MAX_CLIENTS)
      {
        clientActive[v] = WIFI_NOT;
        // debug
        //         usb_send_str("#lost wifi client\r\n");
        // debug end
      }
      // debug
      else if (localEcho)
      {
        usb_send_str("# CLOSED failed to close wifi client\r\n");
      }
      // debug end
    }
    else if (strstr(serRxBuf, "+CIFSR:STAIP"))
    {
      if (localEcho)
        usb_send_str("# got a +CIFSR:STAIP - reading IP\r\n");
      p1 = strchr(serRxBuf, '"');
      if (p1 != NULL)
      {
        for (int i = 0; i < 4; i++)
        {
          p1++;
          wifiIP[i] = strtol(p1, &p1, 10);
        }
        status = WFS_ALL_OK;
      }
    }
    else if (strstr(serRxBuf, "+CIFSR:STAMAC"))
    {
      if (localEcho)
        usb_send_str("# got a +CIFSR:STAMAC - reading MAC\r\n");
      p1 = strchr(serRxBuf, '"');
      strncpy(wifiMAC, ++p1, 18);
      wifiMAC[17] = '\0';
      // no space allowed in MAC - 
      // happens sometime (communication error from 8266)
      for (int i = 0; i < 17; i++)
        if (wifiMAC[i] <= ' ')
          wifiMAC[i] = '_';
    }
    else if (p2)
    {
      //       usb_send_str("# got an OK\r\n");
      //       usb_send_str(serRxBuf);
      //       usb_send_str("# (OK)\r\n");
//       snprintf(s, MSL, "# OK: '%s'\r\n", serRxBuf);
//       usb_send_str(s);
      if (strstr(serRxBuf, "SEND OK"))
      {
        // debug
//         uint32_t t = hbTimerCnt - wifiTiming;
//         snprintf(s, MSL, "# wifi send OK %lu ms\r\n", t);
//         usb_send_str(s);          
        // debug end
        sendingState = WFD_SEND_OK;
        // count good messages
        wifiMsgGood++;
      }
      replyType = WFI_OK;
    }
    else if (strstr(serRxBuf, "ERROR") or strstr(serRxBuf, "FAIL"))
    {
      if (localEcho)
      {
        snprintf(s, MSL, "# err: '%s'\r\n", serRxBuf);
//         usb_send_str("# got an ERROR/FAIL\r\n");
//         usb_send_str(serRxBuf);
        usb_send_str(s);
      }
      replyType = WFI_ERR;
    }
    else if (strstr(serRxBuf, "busy"))
    {
      if (localEcho)
      {
        snprintf(s, MSL, "# busy: '%s'\r\n", serRxBuf);
//         usb_send_str("# got an busy...\r\n");
//         usb_send_str(serRxBuf);
//         usb_send_str("# (BUSY)\r\n");
        usb_send_str(s);
      }
      replyType = WFI_BUSY;
    }
    else if (strstr(serRxBuf, "link is not valid"))
    {
      //       usb_send_str("# got no client (link not valid)...\r\n");
      //       usb_send_str(serRxBuf);
      //       usb_send_str("# (no link)\r\n");
      replyType = WFI_LINK_NOT_VALID;
      // if a send is pending, then abort it
      sendingState = WFD_NONE;
    }
    else if (replyType != WFI_FINISHED)
    { // got data - and not waiting for a reply
      replyType = WFI_MESSAGE;
      // debug
      //       usb_send_str("# got message line:\r\n");
      //       usb_send_str(serRxBuf);
      // debug end
      if (strstr(serRxBuf, "WIFI DISCONNECT"))
      {
        status = WFS_NO_CONNECTION;
        // all clients just died
        for (int i = 0; i < WIFI_MAX_CLIENTS; i++)
          clientActive[i] = WIFI_NOT;
        // send status to USB
        requestingClient = -1;
        sendStatusWiFi();
      }
      else if (strstr(serRxBuf, "WIFI CONNECTED"))
      {
        status = WFS_CONNECTED;
        // send status to USB
        requestingClient = -1;
        sendStatusWiFi();
      }
      else if (strstr(serRxBuf, "WIFI GOT IP"))
      {
        status = WFS_GOT_IP;
        // send status to USB
        requestingClient = -1;
        sendStatusWiFi();
      }
      else 
      {  // unknown message
        //         const int MSL = 75;
        //         char s[MSL];
        //         snprintf(s, MSL, "# wifi got unknown 0x%x 0x%x 0x%x 0x%x (%d):'%s'\r\n", 
        //                  serRxBuf[0], serRxBuf[2], serRxBuf[3], serRxBuf[4], serRxBufCnt, serRxBuf);
        //         usb_send_str(s);
        if (false)
        {
          snprintf(s, MSL, "# wifi 8266 1 reply %s\r\n", serRxBuf);
          usb_send_str(s);
        }
      }
    }
    else
    { // unknown just show result ignore
      if (false)
      {
        snprintf(s, MSL, "# wifi 8266 2 reply %s\r\n", serRxBuf);
        usb_send_str(s);
      }
    }
    //     if (localEcho == 1)
    //     {
    //       usb_send_str("\r\n>>");
    //       //usb_serial_flush_output();
    //     }
    // flush remaining input
    serRxBufCnt = 0;
  }
  else if (n < ' ' and n > '\0')
  { // all remaining characters
    //     if (n != '\r' and n != '\n')
    // debug
    //     const int MSL = 70;
    //     char s[MSL];
    //     snprintf(s, MSL, "# got cnt=%d, a %d \\n=%d, \\r=%d\r\n", serRxBufCnt, n, '\n','\r');
    //     usb_send_str(s);
    // debug end
    if (serRxBufCnt > 0 and serRxBuf[serRxBufCnt-1] == n)
    { // ignore carriage return always and newline as first character
      serRxBufCnt--;
      if (n != '\r')
      {
        // debug
        snprintf(s, MSL, "# wifi got cnt=%d, a %d \\r=%d, \\n=%d\r\n", serRxBufCnt, n, '\r','\n');
        usb_send_str(s);
        // debug end
      }
    }
  }
  else if (serRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    serRxBuf[serRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    usb_send_str(msg);
    serRxBufCnt = 0;
  }
  serRxBuf[serRxBufCnt] = '\0';
  return result;
}

/**
 * send string to wifi socket client.
 * If last message still waits to be send, then nothing is send and function returns false.
 * \param link is client number to send to [0..4]
 * \param msg is string - maximum 100 chars 
 * \param addCRLF ass line feed and carriage return at end 
 * \returns true if send (buffered for send) */
bool UWifi8266::wifiSend(int link, const char * msg, int cnt, bool addCRNL)
{
  const int MSL=110;
  char s[MSL];
  int f = Serial1.availableForWrite(); // free write space
  bool almostSend = false;
  if (sendingState == WFD_SEND_REQ or sendingState == WFD_SENDING)
  { // waiting to send last message
    wifiMsgLost++;
    //     snprintf(s, MSL, "# %lu ms wifi busy - not send %s", hbTimerCnt, msg);
    //     usb_send_str(s);
  }
  else if (cnt >= TX_BUF_SIZE - 2)
  {
    wifiMsgLost++;
    usb_send_str("# line too big for wifi (>400) - not send\r\n");
  }
  else
  {
    if (f > (cnt + 2))
    { // save message in tx buffer - for tx when ready
      strncpy(serTxBuf, msg, cnt);
      if (addCRNL)
      { // add carrage return and line feed
        serTxBuf[cnt++] = '\r';
        serTxBuf[cnt++] = '\n';
      }
      // ensure to terminate
      serTxBuf[cnt] = '\0';
      // get count of characters to send
      //int n = strlen(serTxBuf);
      // prepare send mode command
      snprintf(s, MSL, "AT+CIPSEND=%d,%d\r\n", link, cnt);
      // send request to send
      Serial1.write(s);
      // flag ready to send when OK
      sendingState = WFD_SEND_REQ;
      // start time out counter
      waitForSendOKtime = hbTimerCnt;
      // debug
      // usb_send_str("# send to wifi ready flag set\r\n");
      // debug end
      almostSend = true;
    }
    else
    {
      snprintf(s, MSL, "# wifi no space need %d has %d (skip)\r\n", cnt, f);
      usb_send_str(s);
      wifiMsgLost++;
    }
  }
  return almostSend;
}

void UWifi8266::eePromLoadWifi()
{
  uint8_t n = eeConfig.readByte();
  eeConfig.readBlock(wifiSSID, n);
  wifiSSID[n] = '\0';
  // read also password
  int m = eeConfig.readByte();
  if (m > 0 and m <= 16)
  {
    eeConfig.readBlock(wifiPW, m);
    wifiPW[m] = '\0';
  }
  else
  {
    wifiPW[0] = '\0';
    if (m > 0)
      usb_send_str("# wifi ee load error\r\n");
  }
  portNumber = eeConfig.readWord();
  // debug
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "#ee SSID %d bytes '%s', PW %d bytes '%s'\r\n", n, wifiSSID, m, wifiPW);
  usb_send_str(s);
  // debug end
}

void UWifi8266::eePromSaveWifi()
{ // save port number and SSID
  int n = strlen(wifiSSID);
  // write number of bytes in SSID
  // debug
//   const int MSL = 60;
//   char s[MSL];
//   snprintf(s, MSL, "# wifi ee save n=%d ssid='%s'\r\n", n, wifiSSID);
//   usb_send_str(s);
  // debug end
  eeConfig.pushByte(n);
  eeConfig.pushBlock(wifiSSID, n);
  // password
  n = strlen(wifiPW);
  // write number of bytes in PW
  eeConfig.pushByte(n);
  if (n > 0)
    eeConfig.pushBlock(wifiPW, n);
  // write port number
  eeConfig.pushWord(portNumber);
}
