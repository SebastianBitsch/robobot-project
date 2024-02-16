/*
 *Joystick control 
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

#ifndef UJOY_H
#define UJOY_H


#include <sys/time.h>
#include <cstdlib>
#include "urun.h"
#include "uregbot.h"
#include "utime.h"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

class UPlay;
class URegbot;

class UJoy : public URun
{
  
public:
  /** constructor */
  UJoy(UTeensy * regbot, const char * jsDev);
  /** destructor */
  ~UJoy();
  /**
   * Run joystick motoring */
  void run();
  /**
   * Test for manuel override, and do RC control if needed
   * \param newVel set to true, of velocity has changed (to save messages)
   * \returns true if in manuel control */
  bool makeRegbotRCcontrol(bool newVel);
  
  void setHandler(UHandler* messageHandler)
  {
    handler = messageHandler;
    // initialize messages send from joystick
    initJoyMessageTypes();
  }
  
  
public:
  /** is mission overwritten by manual override */
  bool manOverride = false;
  /** Control forward velocity in m/s */
  float velocity = 0.0;
  /** Control curvature in m^-1 */
  float turnVelocity = 0.0;
  /** Servo 2 position (big servo) -1000..1000 */
  float servo2Pos;
  
  volatile bool joyRunning; // flag for device present
private:
  /**
   * Open joustick device,
   * \returns false if device not found */
  bool initJoy();
  /**
   * Get fresh data from joystick 
   * \return false if device dissapeared of received other than event data */
  bool getNewJsData();
  /**
   * Decode relevant axis and buttons, and
   * translate to robot commands */
  void joyControl();
  /**
   * Enter state with automatic control */
  void enterAutoState();
  /**
   * Enter state with manual control, and velocity == 0 */
  void enterManualState();
  /**
   * send RC messages to robot 
   * \param newVel set to true, if velocity has changed
   *               new velocity command is send only if changed from last message */
  void runManualControl(bool newVel);
  /**
   * Shift to manuel RC control */
  void initManualControl();
  /**
   * Shift back to auto mode */
  void stopManualControl();
  /**
   * send js message to clients
   * \returns true if new event */
  bool sendJsMessage();
  /**
   * Send name and type of message to data handler */
  void initJoyMessageTypes();
  /**
   * get a snapshot of CPU temp */
  void measureCPUtemp();
  
private:
  // pointer to regbot interface
  UTeensy * bot = NULL;
  UHandler* handler = NULL;
  // device
  const char * joyDevice = "/dev/input/js0";
  int jDev = 0;  ///File descriptors
  // manual control flag to avoid too mant toggles
  char toggleState = 0;
  /// are we running in fast mode = 1.0, otherwise a bit slower with this factor
  float fastScale = 0.5;
  //Structure to hold joystick values
  struct jVal {
    bool button[16];
    bool buttonOld[16];
    int axes[16];
  };
  struct jVal joyValues;
  char number_of_axes = 8, number_of_buttons = 11;
  /** old servo position */
  float servo2Pos_old;
  
  static const int MAX_MSG_LENGTH = 400;
  char lastMessage[MAX_MSG_LENGTH] = "";
  /**
   * is regbot set in manuel control */
  bool inManualControl = false;
  /**
   * extra thread to play background music */
  UPlay * play;
public:
  /*
   * CPU temperature in degrees */
  float cputemp;
  
};

#endif
