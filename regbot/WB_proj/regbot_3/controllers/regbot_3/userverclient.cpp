/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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

//#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include "userverclient.h"
#include "userverqueue.h"
#include "userverport.h"

///////////////////////////////////////////////

UServerClient::UServerClient(int index)
{
  conn = -1;
  connected = false;
  msgReceived = 0;
  msgSend = 0;
  tcpipReceived = 0;
  msgBufCnt = 0;
  msgSkippedBytes = 0;
  clientIndex = index;
  clientSerial = -1;
  pthread_mutex_init(&txlock, NULL);
  logReply = false;
  logTimestamp = true;
  serverAlive = NULL;
  queueRawData = false;
}

///////////////////////////////////////////////

UServerClient::~UServerClient()
{
}


///////////////////////////////////////////////

bool UServerClient::justConnected()
{ // should be overwritten if
  // some initialization is required
//   const int MNL = 500;
//   char logName[MNL];
//   snprintf(logName, MAX_FILENAME_LENGTH, "%s_client_%d", appName, clientIndex);
  logCmd.setLogName("socket_server_test.log");
  return true;
}

///////////////////////////////////////////////

bool UServerClient::blockSend(const char * buffer, int length, int msTimeout)
{ // returns true if send and false if connection timeout
  const int pollTime = 5; // miliseconds
  bool result = true;
  int d = 0, n, t = 0;
  // status out is not used, if no error, then just try
  if (connected)
  { // still connected (no error yet)  buffer[4]
    // send length bytes
    while ((d < length) and (t < msTimeout) and result)
    { // get status
      n = send(conn, &buffer[d], length - d, MSG_DONTWAIT |
                                              MSG_NOSIGNAL);
      if (n < 0)
      { // error - an error occurred while sending
        switch (errno)
        {
          case EAGAIN:
            //not all send - just continue
            //printf("UServerClient::blockSend: waiting - nothing send %d/%d\n", d, length);
            Wait(0.005);
            break;
          case EFAULT:
            perror("UServerClient::blockSend: EFAULT: ");
            connected = false;
            break;
          case EINTR:
            perror("UServerClient::blockSend: EINTR: ");
            connected = false;
            break;
          case EINVAL:
            perror("UServerClient::blockSend: EINVAL: ");
            connected = false;
            break;
          case EPIPE:
            perror("UServerClient::blockSend: EPIPE: ");
            connected = false;
            break;
          default:
            perror("UServerClient::blockSend (continues): ");
            result = false;
            break;
        }
        // dump the rest on most errors
        if (not connected)
          result = false;
        if (not result)
          break;
      }
      else
        // count bytes send
        d += n;
      t += pollTime;
    }
    if (not connected)
    {
      shutdown(conn, SHUT_RDWR);
      close(conn);
      connectionLost();
      if (logReply)
      {
        const char * m = "connection lost - last message not send";
        logWrite(buffer, length, true);
        logWrite(m, strlen(m));
      }
    }
  }
  else
    result = false;
  // count messages
  if (result)
    result = t < msTimeout;
  if (result)
    msgSend++;
  if (result and logReply)
    logWrite( buffer, length, true);
  // debug
  if (not result)
  {
    const int MSL = 40;
    char s[MSL];
    int n = mini(MSL - 1, length);
    strncpy(s, buffer, n);
    s[n] = '\0';
    printf("UServerClient::blockSend: failed to send %d chars: '%s'\n", length, s);
  }
  // debug end
  //
  return result;
}

////////////////////////////////////////////////////////

bool UServerClient::initConnection(int clientNumber, int clnt, struct sockaddr_in from,
                                   UServerInQueue * queue,
                                  UTime * aliveTime)
{ // connection info
  clientInfo = from;
  conn = clnt;
  connected = true;
  // statistics init
  msgSend = 0;
  tcpipReceived = 0;
  msgReceived = 0;
  msgBufCnt = 0;
  rxQueue = queue;
  msgBufTime.Now();
  serverAlive = aliveTime;
  connectTime.now();
  // send a welcome message
  justConnected();
  //
  //
  return connected;
}

////////////////////////////////////////////////////////

char * UServerClient::getClientName()
{
  char * result = NULL;
  //
  if (connected)
    result = inet_ntoa(clientInfo.sin_addr);
  //
  return result;
}

///////////////////////////////////////////////

void UServerClient::stopConnection(bool sendHUP, const char * namespaceName)
{
//   const int MRL = MAX_SML_NAME_LENGTH + 3;
//   char reply[MRL];
  //
  if (connected)
  {
    if (sendHUP)
    { // send hup to client
//       if ((clientNamespaceLevel > 0) and (namespaceName != NULL))
//       { // send close message
//         snprintf(reply, MRL, "</%s>\n", namespaceName);
//         blockSend(reply, strlen(reply), 100);
//       }
      // wait for possible reply
      Wait(0.1);
      shutdown(conn, SHUT_RDWR);
    }
    close(conn);
    connected = false;
    if (not connected)
      connectionLost();
  }
}

////////////////////////////////////////////////

bool UServerClient::receiveData()
{ // data is available
  bool result = true;
  const float UNUSED_MESSAGE_PART_TIMEOUT = 10.0; // seconds
  int len;
  UTime t;
  //
  // test for old
  if (msgBufCnt > 0)
  { // test if buffer is too old to use
    t.Now();
    if ((t - msgBufTime) > UNUSED_MESSAGE_PART_TIMEOUT)
    { // skip old remainings
      msgSkippedBytes += msgBufCnt;
      msgBufCnt = 0;
    }
  }
  // get position in buffer to fill new data
  len = MAX_MESSAGE_LENGTH_TO_CAM * 2 - msgBufCnt;
  // get data
  len = recv(conn, &msgBuff[msgBufCnt], len, 0);
  if (len > 0)
  { // data received
    msgBufCnt += len;
    // terminate if string
    msgBuff[msgBufCnt] = '\0';
    //
    // debug
    // printf("UServerClient::receiveData cnt:%d (buff:'%s')\n", msgBufCnt, msgBuff);
    // debug end
    //
    // set receive-time
    msgBufTime.Now();
    // process message
    gotNewMessage();
    // statistics count
    tcpipReceived++;
  }
  else if (len == 0)
  { // closed, e.g. HUP received
//     stopConnection(false, NULL);
//     result = false;
    Wait(0.005);
  }
  else // len = -1
  { // not a message -  other signal that retry
    stopConnection(false, NULL);
    result = false;
  }
  return result;
}


////////////////////////////////////////////////////////

int UServerClient::findEndTag(const char * fromPos, int size)
{
  int used = 0;
  const char * c;
//  int i;
  const char * p1;
  //
  c = fromPos;
  if (*c == '<')
  { // is true XML, so expect a '>' at the end
    p1 = strchr(c, '>');
    if (p1 != NULL)
      used = p1 - c + 1;
    else
    { // test for new line too
      p1 = strchr(c, '\n');
      if (p1 != NULL)
        used = p1 - c + 1;
    }
    // else wait for more
  }
  else
  { // message is just a line, and thus must end in a newline
    p1 = strchr(c, '\n');
    if (p1 != NULL)
      used = p1 - c + 1;
    else
    { // wait for more
      // printf("no \\n to terminate message '%s'\n", c);
    }
  }

  return used;
}

////////////////////////////////////////////////////////

void UServerClient::gotNewMessage()
{ // Got new message from client for parsing
  int used; //, n; //, m;
//   bool trapped;
  //
  while (true)
  { // queue or process - normal server uses process
    if (queueRawData)
    {
      used = rxQueue->addMessage(clientIndex, msgBuff, msgBufCnt, true);
      if (isLogOpen())
      {
        logWrite(msgBuff, used);
      }
    }
    else
    {
      // split into individual messages to queue
      // see if an end-tag is present
      used = findEndTag(msgBuff, msgBufCnt);
      while ((msgBuff[used] <= ' ') and (msgBufCnt > used) )
        // advance past possible '\n' after end tag and surplus whitespace
        used++;
      //
      if (used > 1)
      { // normal text command - queue or use
        used = rxQueue->addMessage(clientIndex, msgBuff, used, false);
        logWrite( msgBuff, used);
        msgReceived++;
      }
      else if (msgBufCnt > MAX_MESSAGE_LENGTH_TO_CAM)
      { // if buffer is half filled, but no message is found
        // then a (much) tool long text message or a data error,
        // skip one byte and try again
        used = 1;
        // count error bytes
        msgSkippedBytes++;
      }
    }
    // part of buffer may now be used
    if (used >= msgBufCnt)
    {
      if (used > msgBufCnt)
      { // should not occur
        printf("UServerClient::gotNewMessage buffer use error! used %d chars, "
               "but buffer has %d chars! (dumping buffer)\n", used, msgBufCnt);
        printf(" - buffer is '%s'\n", msgBuff);
      }
      msgBufCnt = 0;
      break;
    }
    else if (used > 0)
    { // remove unused part of message (and terminationg zero) forward
        memmove(msgBuff, &msgBuff[used], msgBufCnt - used + 1);
        msgBufCnt -= used;
    }
    else
      // message not complete -- wait for more
      break;
  }
}

//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////

void UServerClient::print(char * preStr)
{
  const int SL = 50;
  char s[SL];
  //
  printf("%s", preStr);
  if (isActive())
    printf("is active");
  else
    printf("is closed");
  msgBufTime.getTimeAsString(s, true);
  printf(" (%s) last rx %s rxMsg %2d txCnt %2d skip'd %2d\n",
         getClientName(),
    s, msgReceived, msgSend, msgSkippedBytes);
  snprintf(s, strlen(preStr) + 1, "                        ");
//   if (strlen(clientNamespaceMsg) > 0)
//   {
//     printf("%s", s);
//     printf("namespace: '%s'\n", clientNamespaceMsg);
//   }
}

///////////////////////////////////////////////////////////


void UServerClient::sendAliveReply()
{
  if (tryLock())
  {
    const int MRL = 50;
    char reply[MRL];
    snprintf(reply, MRL, "<alive last=\"%f\"/>\n",
              serverAlive->getTimePassed());
    blockSend(reply, strlen(reply), 30);
    unlock();
  }
}

/////////////////////////////////////////////

void UServerClient::sendStreamRequest(const char * reqTyp)
{
  const char * poseReq = "<stream data=\"$odotime $odox $odoy $odoth\"/>\n";
  int n;
  //
  if (strcmp(reqTyp, "posetime") == 0)
  { // sent request for streaming.
    n = strlen(poseReq);
    lock();
    blockSend(poseReq, n, 50);
    unlock();
  }
}

/////////////////////////////////////////////

void UServerClient::connectionLost()
{
  // special function for odo-pose push from MRC
//   resLink->connectionLost(clientIndex);
}

/////////////////////////////////////////////

bool UServerClient::logOpen()
{
  const int MSL = 100;
  char s[MSL];
  //
  logCmd.closeLog();
  logCmd.openLog();
  if (logCmd.isOpen())
  {
    printf("UServerClient::logOpen: logfile opened client %d to %s\n", clientIndex, getLogFilename());
    snprintf(s, MSL, "#logfile opened from client %d name %s\n", clientIndex, getClientName());
    logWrite(s, strlen(s), true);
  }
  return logCmd.isOpen();
}

/////////////////////////////////////////////

void UServerClient::logClose()
{
  logLock.lock();
  logCmd.closeLog();
  logTimestamp = true;
  logReply = false;
  logLock.unlock();
  printf("UServerClient::logClose: %s closed, client %d\n", getLogFilename(), clientIndex);
}

/////////////////////////////////////////////

bool UServerClient::logWrite( const char * data, const int dataCnt, bool isReply)
{
  bool result;
  const int MFL = 100;
  char sf[MFL];
  UTime t;
  const char * sIsReply = "<-";
  const char * sIsCmd =   "->";
  const char * direction = sIsCmd;
  //
  logLock.lock();
  result = (logCmd.isOpen());
  if (result)
  {
    if (logTimestamp)
    {
      t.now();
      t.getTimeAsString(sf);
      if (isReply)
        direction = sIsReply;
      fprintf(logCmd.getF(), "\n¤ %s %lu.%6lu (%s) %d chars:\n", direction, t.getSec(), t.getMicrosec(), sf, dataCnt);
    }
    snprintf(sf, MFL, "%%%ds", dataCnt);
    fprintf(logCmd.getF(), sf, data);
  }
  logLock.unlock();
  return result;
}


