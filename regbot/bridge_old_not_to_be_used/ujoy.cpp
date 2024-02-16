/***************************************************************************
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

#include <sys/ioctl.h>
#include <signal.h>
#include <linux/joystick.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "uhandler.h"
#include "ujoy.h"

using namespace std;

// start is green button
#define BUTTON_GREEN   0
// red button is 1
#define BUTTON_RED   1
// blue button is 2
#define BUTTON_BLUE   2
// image is yellow button (3)
#define BUTTON_IMAGE   3
#define BUTTON_YELLOW   3
// left front button is 4
// right front button is fast
#define BUTTON_FAST    5
// goto manuel mode is all 3 upper-center buttons
#define BUTTON_BACK 6
#define BUTTON_START 7
#define BUTTON_MANUAL 8
// axis 0 is left hand left-right axis
// axis 1 is left hand up-down axis
#define AXIS_SERVO_2   1
// axis 2 is left front speeder
// axis 3 is right hand left-right
#define AXIS_TURN      3
// axis 4 is right hand up-down
#define AXIS_VEL       4
// axis 5 is right front speeder
// axis 6 id digital left-write
// axis 7 is digital up-down
// parameters for JOY control (velocity is in m/s)
#define VELOCITY_MAX          2.5
#define VELOCITY_TURN_MAX     0.8
#define VELOCITY_SERVO_MAX   10.0
// factor for normal speed
#define VELOCITY_SLOW_FACTOR  0.33


class UPlay : public URun
{
public:
  void run()
  { // start the playing - with low priority (nice 14)
    printf("playing (requires libsox-fmt-mp3 to be installed) \n");
    // nice gives low priority to music process
    // -v0.1 gives low amplitude (10%)
    // music.mp3 is a symbolic link to some music
    system("nice -n14 play -v0.1 ~/Music/music.mp3");
    printf("play finished\n");
    runFinished();
  }

  bool isPlaying()
  {
    int e = system("pgrep play");
    return e == 0;
  }
  
  void stopPlaying()
  { // kill the play process (if running)
    system("pkill play");
    // join the process (if running)
    stop();
  }
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


UJoy::UJoy(UTeensy * regbot, const char * jsDev)
{ /** robot interface */
  bot = regbot;
  joyDevice = jsDev;
  /** servo position in range -1000 to 1000 */
  servo2Pos = 0.0;
  servo2Pos_old = 0.0;
  handler = NULL;
  start();
  // object to play music (if blue button is pressed and the file exist)
  play = new UPlay();
}
/** destructor */
UJoy::~UJoy()
{
  th1stop = true;
  printf("Joy stopping\n");
  if (play->isPlaying())
    play->stopPlaying();
}


/////////////////////////////////////////////////////////////////////////////////

/** read form joystick driver */
bool UJoy::getNewJsData()
{
  struct js_event jse;
  bool lostConnection = false;
  bool isOK = false;
  // read full struct or nothing
  int bytes = read(jDev, &jse, sizeof(jse));
  // detect errors
  if (bytes == -1) 
  { // error - an error occurred while reading
    switch (errno)
    { // may be an error, or just nothing send (buffer full)
      case EAGAIN:
        //not all send - just continue
//         printf("UJoy::getNewJsData: waiting - got EAGAIN (%d bytes)\n", bytes);
        usleep(100);
        break;
      default:
        perror("UJoy::getNewJsData (other error device error): ");
        lostConnection = true;
        break;
    }
  } 
  if (lostConnection)
  {
    joyRunning = false;
    if (jDev) 
    {
      close(jDev);
      jDev = -1;
      sendJsMessage();
    }
  }
  if (joyRunning and bytes > 0)
  {
    if (bytes != sizeof(jse) and bytes > 0) 
    { // size error
      printf("JOY control : Unexpected byte count from joystick:%d - continues\n", bytes);
    } 
    else 
    { //Proper joystick package has been recieved
      //Joystick package parser
      isOK = true;
      jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
      switch(jse.type) {
        // changed axis position
        case JS_EVENT_AXIS:
          if (jse.number < 16) {
            joyValues.axes[jse.number] = jse.value;
          }
          break;
        // changed button state
        case JS_EVENT_BUTTON:
          if (jse.number < 16) {
            joyValues.button[jse.number] = jse.value;
          }
          break;
        default:
          printf("UJoy::getNewJsData : got bad data (event=%d, time=%d) - ignores\n", jse.type, jse.time);
          isOK = false;
          break;
      }
//       printf("UJoy::getNewJsData : got %d bytes (event=%d, time=%d, value=%d) - debug\n", bytes, jse.type, jse.time, jse.value);
    }
  }
  return isOK;
}

//////////////////////////////////////////////////////////

void UJoy::initManualControl()
{
  printf("JOY:: in manuel control\n");
  //bot->send("rc=1 0 0\n");  
  char s[20] = {"rc=1 0 0\n"};
  handler->handleCommand(CLIENT_JOY, s, false);  
  //   usleep(10000);
//   // enable servos for steering and big servo
//   bot->send("svo 1 0 0  1 0 0  0 0 0  0 0 0  0 0 0\n");
//   usleep(10000);
//   // enable servo 1 as steering
//   bot->send("sv1 1 0 0.16 90.0\n");
}

//////////////////////////////////////////////////////////

void UJoy::runManualControl(bool newVel)
{
  const int MSL = 50;
  char s[MSL];
  // test servo position
  if (fabs(joyValues.axes[AXIS_SERVO_2]) > 500)
  { // move servo
    servo2Pos += float(joyValues.axes[AXIS_SERVO_2]) / 32000.0 * VELOCITY_SERVO_MAX * fastScale;
    if (servo2Pos > 1000)
      servo2Pos = 1000;
    else if (servo2Pos < -1000)
      servo2Pos = -1000;
    // debug
//     printf("Joy::axis2: %d (servo 2 pos = %g)\n", joyValues.axes[AXIS_SERVO_2], servo2Pos);
    // debug end
  }
  if (fabs(servo2Pos - servo2Pos_old) > 2.0)
  {
    snprintf(s, MSL, "servo 2 %d\n", int(round(servo2Pos)));
    handler->handleCommand(CLIENT_JOY, s, false);
//     bot->send(s);
    
    servo2Pos_old = servo2Pos;
//     printf("Joy:: %.3f RC servo control send: %s", bot->getTime(), s);
  }
  else if (newVel)
  { // send velocity and turn
    snprintf(s, MSL, "rc=1 %g %g\n", velocity, turnVelocity);
    handler->handleCommand(CLIENT_JOY, s, false);
//     bot->send(s);
    //   printf("Mission:: RC control send: %s", s);
  }
}

//////////////////////////////////////////////////////////

void UJoy::stopManualControl()
{
  char s[20] = {"rc=0 0 0\n"};
  handler->handleCommand(CLIENT_JOY, s, false);
//   bot->send("rc=0 0 0\n");  
  printf("JOY:: return to auto mode\n");
}

/////////////////////////////////////////////////////////////////////////////////

bool UJoy::makeRegbotRCcontrol(bool newVel)
{
  if (manOverride)
  { // do not run automatic mission
    if (not inManualControl)
    { // just entered manual control 
      initManualControl();
      inManualControl = true;
    }
    else
      runManualControl(newVel);
  }
  else if (inManualControl)
  {
    stopManualControl();
    inManualControl = false;
  }
  return inManualControl;
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::joyControl()
{ // called when js driver sees a change (may be slow)
  if (joyValues.button[BUTTON_FAST])
    fastScale = 1.0;
  else
    fastScale = VELOCITY_SLOW_FACTOR;
  // velocity
  if (fabs(joyValues.axes[AXIS_VEL]) > 500)
  { // velocity is valid
    float velVector;
    velVector = joyValues.axes[AXIS_VEL];
    velocity = -velVector / 32000.0 * fastScale * VELOCITY_MAX;
  }
  else
    velocity = 0.0;
  // turn
  if (fabs(joyValues.axes[AXIS_TURN]) > 500)
  { // velocity is valid
    turnVelocity = -float(joyValues.axes[AXIS_TURN]) / 32000.0 * VELOCITY_TURN_MAX;
  }
  else
    turnVelocity = 0.0;
  // start
  if (joyValues.button[BUTTON_GREEN])
  { // fake that start is pressed (as if came from console or any other client)
    char s[20] = {"<event=33\n"};
    handler->handleCommand(CLIENT_CONSOLE, s, false);
//    bot->setEvent(33);
    // exit remote control state
    enterAutoState(); 
  }
  if (joyValues.button[BUTTON_BLUE])
  {
    if (play != NULL)
    {
      if (play->isPlaying())
        play->stopPlaying();
      else
        play->start();
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::enterAutoState()
{
  manOverride = false;
  fprintf(stdout, "Joy::Enter auto mode\n");
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::enterManualState()
{
  manOverride = true;
  fprintf(stdout, "Joy::Enter manual mode\n");
}

void UJoy::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    float t = 0;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
      cputemp = t/1000.0;
    }
    //printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s); 
    fclose(temp);
  }
  else
    printf("#failed to open temp-file");
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::run()
{
  int events = 0;
  bool newEvent = false;
  sleep(1);
  joyRunning = initJoy();  
  if (not joyRunning)
  { // no such device
    fprintf(stderr, "Can't open joystick port: %s\n",joyDevice);
  }
  while (handler == NULL)
    usleep(100000);
  sendJsMessage();
  UTime t;
  t.now();
  bool initMessages = true;
  while (not th1stop)
  {
    if (t.getTimePassed() > 2)
    { // tome to measure CPU temp
      measureCPUtemp();
      t.now();
    }
    if (not joyRunning)
    {
      sleep(2);
      // retry
      joyRunning = initJoy();  
      t.now();
      initMessages = true;
      sendJsMessage();
    }
    else
    { // Device is present
      if (initMessages)
      {
        if (t.getTimePassed() > 1.0)
          initMessages = false;
      }
      bool gotEvent= getNewJsData();
      if (gotEvent and not initMessages)
      { //Detect manual override toggling
        if ((joyValues.button[BUTTON_MANUAL] == 1) and not toggleState)
        {
          if (manOverride) 
            enterAutoState();
          else
            enterManualState();
          //Set state for toggleing - wait for button release
          toggleState = 1; 
        } 
        else if (joyValues.button[BUTTON_MANUAL] == 0)
        {
          toggleState = 0;
        }
        if (manOverride)
        { // we are in manual mode, so
          // generate robot control messages
          joyControl();
        }
        // send also normal joystick status to clients
        if (handler != NULL)
          newEvent = sendJsMessage();
        if (newEvent)
        {
          events++;
          if (false)
            printf("Joy:: event (man=%d) event=%d\n", manOverride, events);
        }
        // save old button values
        for (int i = 0; i < 16; i++)
          joyValues.buttonOld[i] = joyValues.button[i];
      }
      else
        usleep(10000);
      if (not initMessages)
        makeRegbotRCcontrol(newEvent);
      newEvent = false;
    }
  }
  if (joyRunning) 
  { // close device nicely
    joyRunning = false;
    if (jDev >= 0) 
      close(jDev);
    jDev = -1;
    // revert to auto mode
    enterAutoState();
  }
}


bool UJoy::initJoy(void) 
{ //Open first serial js port
  jDev = open (joyDevice, O_RDWR | O_NONBLOCK);
  if (jDev >= 0)
  { // Joystic device found
    printf("Opened joystick on port: %s\n",joyDevice);
    //Query and print the joystick name
    char name[128];
    if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
      strncpy(name, "Unknown", sizeof(name));
    printf("Joystick model: %s\n", name);    
    //Query and print number of axes and buttons
    ioctl (jDev, JSIOCGAXES, &number_of_axes);
    ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
    printf("Registrered %d axes and %d buttons on joystick\n",number_of_axes,number_of_buttons);
  }
  return jDev >= 0;
}  
  

void UJoy::initJoyMessageTypes()
{
  char s1[] = "joy set 0 0 Joystick position, running, manual override, Axis count, Button count, A1, A2 ... An, B1, B2 ... Bn";
  handler->handleCommand(CLIENT_JOY, s1, false);
  char s2[] = "rc=set 1 Remote control for robot 'rc=m v t'  m:manual override, v:velocity, t:turn velocity.";
  handler->handleCommand(CLIENT_JOY, s2, false);
}
  
bool UJoy::sendJsMessage()
{
  char s[MAX_MSG_LENGTH];
  char * p1 = s;
  int n = 0;
  snprintf(s, MAX_MSG_LENGTH, "joy %d %d %d %d", joyRunning, manOverride, number_of_axes, number_of_buttons);
  for (int i = 0; i < number_of_axes; i++)
  {
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, MAX_MSG_LENGTH - n, " %d", joyValues.axes[i]);
  }
  for (int i = 0; i < number_of_buttons; i++)
  {
    n += strlen(p1);
    p1 = &s[n];
    snprintf(p1, MAX_MSG_LENGTH - n, " %d", joyValues.button[i]);
  }
  
  n += strlen(p1);
  p1 = &s[n];
  snprintf(p1, MAX_MSG_LENGTH - n, "\r\n");
  bool theSame = strcmp(s, lastMessage) == 0;
  if (not theSame)
  {
//    printf("UJoy::sendJsMessage event %s not equal to--> %s\n", lastMessage, s);
    strncpy(lastMessage, s, MAX_MSG_LENGTH);
    handler->handleCommand(CLIENT_JOY, s, false);
  }
//   else
//     printf("UJoy::sendJsMessage event %s equal to--> %s\n", lastMessage, s);
  return not theSame;
}
