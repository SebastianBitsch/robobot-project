/**************************************************************************
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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "../Downloads/raspicam-0.0.5/src/raspicam.h"

//#define PI_CAM
#ifdef PI_CAM
#include <raspicam/raspicam.h>
#endif
#include "urun.h"
#include "ucamera.h"
#include "uregbot.h"
#include "umission.h"
#include "ujoy.h"

using namespace std;


// void shutDownHandler(int signal)
// { // Close all devices and files
//   if (signal > 0)
//     // print signal message
//     psignal(signal, "Signal");
//   mission.stop();
//   reg.stop();
//   cam.stop();
//   printf("main:: cam stopped\n");
//   joy.stop();
//   printf("main:: joy stopped\n");
// //   if (signal > 0)
// //     // stop process now!
// //     exit(EXIT_SUCCESS);
// }


// void initOther()
// { // crash and quit handler
//   struct sigaction nact, oact;
//   // Registering the interrupt handler, catching
//   /* Set up the structure to specify the new action. */
//   nact.sa_handler = shutDownHandler;
//   sigemptyset (&nact.sa_mask);
//   nact.sa_flags = 0;
//   // catch the following actions
//   sigaction (SIGINT, NULL, &oact);
//   if (oact.sa_handler != SIG_IGN)
//     sigaction (SIGINT, &nact, NULL);
//   sigaction (SIGHUP, NULL, &oact);
//   if (oact.sa_handler != SIG_IGN)
//     sigaction (SIGHUP, &nact, NULL);
//   sigaction (SIGTERM, NULL, &oact);
//   if (oact.sa_handler != SIG_IGN)
//     sigaction (SIGTERM, &nact, NULL);
// }

void printHelp(char * name)
{ // show help
  printf("\nUsage: %s [<from mission> [<to mission>]]\n\n", name);
  printf("<from mission> and <to mission>:\n");
  printf("     number in the range 1..998, and the code\n");
  printf("     run only the mission parts in this range.\n\n");
  printf("E.g.: './%s 2 2' runs mission 2 only\n\n", name);
  printf("NB!  Robot may continue if this app is stopped with ctrl-C.\n\n");
}

////////////////////////////////////////////////////////////////////

int main ( int argc,char **argv ) 
{
  int firstMission = 1, lastMission = 998;
//   initOther();
  // are there mission parameters
  if (argv[1] != NULL and (*argv[1] == '-' or *argv[1] == 'h'))
  {
    printHelp(argv[0]);
  }
  else 
  { // not help - create modules
    URegbot reg;
    UCamera cam(&reg);
    UJoy joy(&reg, &cam);
    UMission mission(&reg, &cam, &joy);
    // get mission range (default is 1..988 (all))
    if (argc > 1)
    { // get mission range
      firstMission = strtol(argv[1], NULL, 10);
      lastMission = firstMission;
      if (argc > 2)
        lastMission = strtol(argv[2], NULL, 10);
    }
    // start requested missions
    if (reg.isConnected)
    { // starting the show - after this small brake
      sleep(1);
      printf("Main starting ...\n");
      mission.runMission(firstMission, lastMission);
      printf("Main is done.\n");
    }
    else
    {
      printf("Main:: no robot found - terminate\n");
    }
  }
}
