/**
 * Interface with regbot
 *  
 * ************************************************************************
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

#include <sys/time.h>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>

#include "uregbot.h"
#include "uhandler.h"

using namespace std;

/** constructor */
UTeensy::UTeensy()
{
    regbotConnectionOpen = false;
  // botComLog = fopen("log_to_from_regbot.txt", "w");
  botComTx = NULL;
  botComRx = NULL;
}
/** destructor */
UTeensy::~UTeensy()
{ // stop all activity before close
  // is done in run()
  closeCommLog();
}

void UTeensy::openCommLog(const char * path)
{
  if (botComRx == NULL)
  {
    botComRx = new ULogFile("log_rx_regbot.txt", path);
    botComRx->openLog();
    if (botComRx->isOpen())
      printf("UTeensy::openCommLog - opened logfile %s\n", botComRx->getLogFileName());
    botComTx = new ULogFile("log_tx_regbot.txt", path);
    botComTx->openLog();
    if (botComTx->isOpen())
      printf("UTeensy::openCommLog - opened logfile %s\n", botComTx->getLogFileName());
  }
}

void UTeensy::closeCommLog()
{
  if (botComRx != NULL)
  {
    delete botComRx;
    botComRx = NULL;
  }
  if (botComTx != NULL)
  {
    delete botComTx;
    botComTx = NULL;
  }
}


void UTeensy::sendMsg(const char * key, const char * params, int msTimeout)
{ // construct string and send to robot
  const int MSL = 500;
  char s[MSL];
  int n = strlen(key) + strlen(params) + 5;
  if (n > MSL)
  {
    printf("UTeensy::send: msg to robot too long (%d>%d) truncated\r\n", n, MSL);
    n = MSL-1;
    s[n] = '\0';
  }
  if (strlen(params) == 0)
    snprintf(s, n, "%s\r\n", key);
  else
    snprintf(s, n, "%s %s\r\n", key, params);
  send(s, msTimeout);
}

/**
  * send a string to the serial port */
void UTeensy::sendString(const char * cmd, int msTimeout)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
  int t = 0;
  bool lostConnection = false;
  sendMtx.lock();
  if (regbotConnectionOpen)
  { // simulator or not
    if (socket.connected)
    { // send string to socket (REGBOT simulator)
      int n = strlen(cmd);
      int m = socket.sendData(cmd);
      if (m < n)
        printf("# failed to send all data to simulator, send %d/%d of '%s'\n", m, n, cmd);
    }
    else
    { // try send string to USB
      int n = strlen(cmd);
      int d = 0;
      while ((d < n) and (t < msTimeout))
      { // want to send n bytes to usbport within timeout period
        int m = write(usbport, &cmd[d], n - d);
        if (m < 0)
        { // error - an error occurred while sending
          switch (errno)
          { // may be an error, or just nothing send (buffer full)
            case EAGAIN:
              //not all send - just continue
              //printf("UnblockedIo::blockSend: waiting - nothing send %d/%d\n", d, length);
              usleep(1000);
              t += 1;
              break;
            default:
              perror("UTeensy::send (closing connection): ");
              lostConnection = true;
              break;
          }
          // dump the rest on most errors
          if (lostConnection)
            break;
        }
        else
          // count bytes send
          d += m;
      }
    }
    // debug logging
    {
      UTime t;
      t.now();
      if (botComTx != NULL)
        botComTx->toLog(t, cmd);
    }
  }
  // 
  //usleep(4000); // Short sleep helps communication be more reliable when many command lines are send consecutively.
  sendMtx.unlock();
  if (lostConnection)
  {
    close(usbport);
        regbotConnectionOpen = false;
    usbport = -1;
  }
  else
    lastTxTime.now();
}
/**
  * receive thread */
void UTeensy::run()
{ // read thread for REGBOT messages
  int n = 0;
  rxCnt = 0;
  timeval idleTime;
  timeval aliveTime;
  bool err = false;
  int readIdleLoops = 0;
  // wait for other classes to initialize
//   printf("UTeensy::run run stated\n");
  usleep(100000);
  gettimeofday(&idleTime, NULL);
  // get robot name
  //send("u4\n");
  while (not th1stop)
  {
    if (not regbotConnectionOpen)
    { // wait a second (or 2)
      sleep(2);
      // then try to connect
      openToRegbot();
      if (regbotConnectionOpen)
      { // request base data
        // sub s p m    Subscribe s=1, unsubscribe s=0, p=priority 0 (high) .. 4(low), "
        //              "m=msg:0=hbt, 1=pose,2=IR,3=edge,4=acc,5=gyro,6=amp,7=vel\r\n");
        send("sub 1 3 0\n"); // hartbeat
//         send("sub 1 1 1\n"); // pose
      }
    }
    else
    { // we are connected
      if (socket.connected)
      { // from simulator
        n = socket.readChar(&rx[rxCnt], &err); // read(usbport, &rx[rxCnt], 1);
        if (err)
        { // read failed (other than just no data)
          perror("Regbot:: socket port error");
          usleep(100000);
          sendMtx.lock();
          // don't close while sending
          socket.closeSocket();
          sendMtx.unlock();
          regbotConnectionOpen = false;
        }
      }
      else
      { // from USB
        n = read(usbport, &rx[rxCnt], 1);
        if (n < 0 and errno == EAGAIN)
        { // no data
          n = 0;
        }
        else if (n < 0)
        { // other error - close connection
          perror("Regbot:: port error");
          usleep(100000);
          sendMtx.lock();
          // don't close while sending
          close(usbport);
          sendMtx.unlock();
          regbotConnectionOpen = false;
          usbport = -1;
        }
      }
      if (n == 1)
      { // got a new character
        rxCnt++;
        if (rx[rxCnt-1] == '\n')
        { // terminate string
          rx[rxCnt] = '\0';
          // handle new line of data
          // debug
          {
            UTime t;
            t.now();
            if (botComRx != NULL)
              botComRx->toLog(t, rx);
          }
          decode(rx);
          // reset receive buffer
          rxCnt = 0;
          n = 0;
          // reset idle time
          gettimeofday(&idleTime, NULL);
  //         printf("*** got:%s\n", rx);
        }
      }
      else if (n == 0)
      {  // no data, so wait a bit
        usleep(1000);
        readIdleLoops++;
      }
      //
      timeval t2;
      gettimeofday(&t2, NULL);
      float dt = getTimeDiff(t2, aliveTime);
      if (dt > 1.0)
      { // waited more than 1s - tell that we are alive
        send("alive\n");
        aliveTime = t2;
        //printf("# REGBOT read idle loops %d in 1 second (sleeping 1ms each)\n", readIdleLoops);
        readIdleLoops = 0;
      }
    } // connected
  }
  if (socket.connected)
    socket.closeSocket();
  else if (usbport != -1)
  {
    close(usbport);
    usbport = -1;
  }
  regbotConnectionOpen = false;
  printf("Regbot:: thread stopped and port closed\n");
}  

/**
  * Open the connection.
  * \returns true if successful */
bool UTeensy::openToTeensy()
{
  if (simPort > 1000)
  { // open connection to simulator
    string s = to_string(simPort);
    socket.createSocket(s.c_str(), simHost);
    socket.tryConnect();
    regbotConnectionOpen = socket.connected;
  }
  else
  { // should use USB port
    //   printf("Regbot::openToRegbot opening\n"); 
    usbport = open(usbdevice, O_RDWR | O_NOCTTY | O_NDELAY);
    if (usbport == -1)
    {
      if (connectErrCnt < 5)
        perror("Regbot::openToRegbot open_port: Unable to open regbot connection");
      connectErrCnt++;
    }
    else
    { // set connection to non-blocking
      int flags;
      if (-1 == (flags = fcntl(usbport, F_GETFL, 0)))
        flags = 0;
      fcntl(usbport, F_SETFL, flags | O_NONBLOCK);    
  // #ifdef armv7l
      struct termios options;
      tcgetattr(usbport, &options);
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; //<Set baud rate
      options.c_iflag = IGNPAR;
      options.c_oflag = 0;
      options.c_lflag = 0;
      tcsetattr(usbport, TCSANOW, &options);
  // #endif
      tcflush(usbport, TCIFLUSH);
      connectErrCnt = 0;
    }
      regbotConnectionOpen = usbport != -1;
  }
  return regbotConnectionOpen;
}

/////////////////////////////////////////////////

/**
  * decode messages from REGBOT */
void UTeensy::decodeFromTeensy(char * msg)
{
  char * p1 = msg;
  // skip whitespace
  while (*p1 <= ' ' and *p1 > '\0')
    p1++;
  // debug
//   printf("Regbot:: got:%s", p1);
  // debug end
  if (*p1 != '\0')
    handler->handleCommand(CLIENT_ROBOT, p1, false);
}

//////////////////////////////////////////////////


void UTeensy::setHandler(UHandler* dataHandler)
{
  handler = dataHandler;
  initMessageTypes();
  start();
}

void UTeensy::initMessageTypes()
{ // set data types
  char s3[] = "servo set 1 Servo control for robot 'servo n p' n:servo, p:position.";
  handler->handleCommand(CLIENT_CONSOLE, s3, false);
  char s4[] = "pse set 0 Robot pose [x [m], y [m], h [rad], tilt [rad]].";
  handler->handleCommand(CLIENT_CONSOLE, s4, false);
  char s5[] = "hbt set 0 Heartbeat [time [sec], battery voltage [V].";
  handler->handleCommand(CLIENT_CONSOLE, s5, false);
}

