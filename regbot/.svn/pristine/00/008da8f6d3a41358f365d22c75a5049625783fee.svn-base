/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
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

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "userverport.h"
#include "ujoy.h"
#include "uhandler.h"
#include "uoled.h"
#include "ubridge.h"

using namespace std;

char ** argvs;


void printHelp()
{ // show help
//   printf("\nBridge between a Regbot robot and joystick and socket clients\n");
//   printf("Runs on Linux PC and raspberry (preferably in rc.local - runs here an oled display too if available).\n");
  printf("%s command-line commands:\n", argvs[0]);
  printf("   q: quit\n");
  printf("   s: status of data items etc\n");
  printf("   h: this help\n");
  printf("   help: help for the bridge\n");
  printf("Lines with more than one character are sent to the handler as a message.\n\n");
}

////////////////////////////////////////////////////////////////////



void restart()
{
  printf("# Going to reboot ...\r\n\n\n\n");
  execve(argvs[0], argvs, NULL);
  printf("# Never here - this process should be overwritten by now.\n");
}

int setNonblocking(int fd)
{
  int flags;
  if (-1 == (flags = fcntl(fd, F_GETFL, 0)))
    flags = 0;
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}     

int setBlocking(int fd)
{
  int flags;
  if (-1 == (flags = fcntl(fd, F_GETFL, 0)))
    flags = 0;
  return fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
}     


bool readCommandLineParameters(int argc, char ** argv, 
                               bool * deamon, 
                               int * port,
                               char usbdev[], int usbdevSize,
                               char jsdev[], int jsdevSize,
                               char simhost[], int simhostSize,
                               int * simport
                              )
{
  // are there mission parameters
  bool isHelp = false;
  *deamon = false;
  *port = 24001;
  *simport = 0;
  strncpy(usbdev, "/dev/ttyACM0", usbdevSize);
  strncpy(jsdev, "/dev/input/js0", usbdevSize);
  strncpy(simhost, "localhost", simhostSize);
  for (int i = 1; i < argc; i++)
  {
    if (isHelp)
      break;
    char * c = argv[i];
    if (c[0] == '-')
    {
      switch (c[1])
      {
        //case '-':
        case 'h':
          isHelp = true;
          break;
        case 'd':
          *deamon = true;
          break;
        case 'p':
          *port = strtol(argv[++i], NULL, 10);
          break;
        case 'i':
          *simport = strtol(argv[++i], NULL, 10);
          break;
        case 'u':
          strncpy(usbdev, argv[++i], usbdevSize);
          break;
        case 's':
          strncpy(simhost, argv[++i], simhostSize);
          break;
        case 'j':
          strncpy(jsdev, argv[++i], jsdevSize);
          break;
        default:
          printf("# Unused parameter '%s'\n", argv[i]);
          break;
      }
    }
    else
      printf("Parameter option should start with a '-', not '%s'\n", c);
  }
  // tell config
  if (not isHelp)
  { // mostly debug
    printf("\n# Using the following parameters\n");
    printf("# usbdev  = %s\n", usbdev);
    printf("# port    = %d\n", *port);
    printf("# gamepad = %s\n", jsdev);
    printf("# simhost = %s\n", simhost);
    printf("# simport = %d\n", *simport);
    printf("# daemon  = %d\n", *deamon);
    printf("# ------------------------\n");
  }
  return isHelp;
}


int main ( int argc,char **argv ) 
{ // save application name for restart
  argvs = argv;
  bool asDeamon = false;
  bool help = false;
  int loop  = 0;
  const int MPL = 100;
  char usbDev[MPL];
  char simHost[MPL];
  char jsDev[MPL];
  int port;
  int simport;
  help = readCommandLineParameters(argc, argv, &asDeamon, &port, usbDev, MPL, jsDev, MPL, simHost, MPL, &simport);
  if (port == simport)
  {
    printf("#### Port can't be the same for simulator input %d and output %d!\n", simport, port);
    help = true;
  }
  if (help)
  {
    printf("#  \n");
    printf("# Bridge from a REGBOT to a socket connection\n");
    printf("# parameters:\n");
    printf("#  -h     : this help\n");
    printf("#  -d     : run as daemon - without listening to keyboard, default is %d\n", asDeamon);
    printf("#  -p N   : set output port number to N (default is %d)\n", port);
    printf("#  -i N   : set input  port number for simulator REGBOT, default is %d (use USB)\n", simport);
    printf("#  -s str : set input host name/IP for simulated REGBOT, default is '%s'\n", simHost);
    printf("#  -u str : set USB device to REGBOT, default is '%s'\n", usbDev);
    printf("#  -j str : set joystick device name, default is '%s'\n", jsDev);
    printf("#  \n");
  }
  else
  { // not help 
    ULogFile::timeToFlush = 0;
    // create modules
    UOled oled;
    // connection to robot
    UTeensy reg;
    reg.setDevice(usbDev, simport, simHost);
    // connection to remote control (joystick)
    UJoy joy(&reg, jsDev);
    // socket server and client handler
    UServerPort server(port);
    // message storage for all data items
    UData data(&server);
    // message handler
    UHandler handler(&reg, &data, &joy, &oled);
    /** responder for updates where this bridge has to do a special response,
    * like show on display or send to robot */
    UBridge bridge(&reg, &oled, &data, &server);
    sleep(1);
    printf("Main:: starting ...\n");
    // distribute pointers
    bridge.setHandler(&handler);
    data.setHandler(&handler, &bridge);
    reg.setHandler(&handler);
    server.setHandler(&handler);
    joy.setHandler(&handler);
    sleep(1);
    string s;
    char c1;
    char c2[2] = {'\0','\0'};
    if (asDeamon)
      printf("Main:: Running as deamon - no keyboard input\n");
    else
    {
      printf("Main:: pres q\\n to quit\n");
      // subscribe to warnings etc
      handler.handleCommand(CLIENT_CONSOLE, (char*)"# subscribe 6", false);
      setNonblocking(stdin->_fileno);
    }
    int sleepLoops = 0;
    while (not quitBridge)
    { // wait for a 'q' from keyboard
      usleep(100000);
      loop++;
      if (loop % 10 == 0)
        ULogFile::timeToFlush++;
      s = "";
      c2[0] = '\0';
      // debug - keep client alive, even if no Regbot
      if (bridge.lastHbt.getTimePassed() > 0.8)
      { // keep client alive with fake data
        const int MSL = 100;
        char s[MSL];
        // hbt time battery ctrl mission rc\n
        snprintf(s, MSL, "hbt %.1f %.1f 0 9 0\n", oled.oldRegbotTime, oled.oldBatteryVoltage);
        handler.handleCommand(CLIENT_CONSOLE, s, false);
      }
      // debug end
      if (asDeamon)
      { // nothing to do
        usleep(200000);
      }
      else
      { // get a line of console command, while watching for 
        // quit or reboot flag
        while (c2[0] != '\n')
        {
          int e = read(stdin->_fileno, c2, 1);
          // getline(cin, s, '\n');
          if (quitBridge)
            break;
          else if (e <= 0)
          { // no data, so wait
            usleep(200);
            sleepLoops++;
          }
          else if (e == 1 and c2[0] >= ' ')
          { // add to command line string
            s.append(c2);
          }
        }
      }
      if (not quitBridge and not asDeamon)
      { // there is a line of text in s
        if (s.length() > 0)
          c1 = s[0];
        if (s.length() == 1)
        {
          if (c1 == 'q')
            quitBridge = true;
          if (c1 == 's')
          {
            printf("Regbot bridge: regbot OK=%d, Joy OK=%d, Oled OK=%d, socket=%d (port %d on %s), clients=%d\n", 
                  reg.regbotConnectionOpen, joy.joyRunning, oled.displayFound, server.open4Connections, server.serverPort, server.hostname, server.getActiveClientCnt()
                  );
            data.printStatus();
            server.printStatus();
            printf("\r\n");
          }
          else if (c1 == 'h')
          {
            printHelp();
          }
        }
        else if (s.length() > 0)
        {
          handler.handleCommand(CLIENT_CONSOLE, (char*)s.c_str(), false);
        }
        printf("bridge >> ");
        fflush(stdout);
      }
    }
    printf("Stopping reg\n");
    reg.stop();
    printf("Stopping joy\n");
    joy.stop();
    printf("Main is leaving object scope\n");
  }
  // reset to blocking mode (to make nano etc work)
  setBlocking(stdin->_fileno);
  printf("Finished\n");
  if (restartBridge)
  {
    printf("Will restart in a moment\n");
    restart();
  }
}

