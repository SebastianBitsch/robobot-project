#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* gamepad test page
 #*
 #*   This program is free software; you can redistribute it and/or modify  *
 #*   it under the terms of the GNU General Public License as published by  *
 #*   the Free Software Foundation; either version 2 of the License, or     *
 #*   (at your option) any later version.                                   *
 #*                                                                         *
 #*   This program is distributed in the hope that it will be useful,       *
 #*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 #*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 #*   GNU General Public License for more details.                          *
 #*                                                                         *
 #*   You should have received a copy of the GNU General Public License     *
 #*   along with this program; if not, write to the                         *
 #*   Free Software Foundation, Inc.,                                       *
 #*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 #***************************************************************************/

import threading
import time

class UJoy(object):
  dataRead = False # for all servos
  lock = threading.RLock()
  lastDataRequestTime = time.time()
  lastDataSetTime = time.time()
  inEdit = False
  lastDataRequest = 0
  lastTab = ""
  inTimerUpdate = True
  gotFirstData = False;
  joyPresent = False
  manualControl = False
  axisCount = 8
  axis = [0,0,0,0,0,0,0,0]
  buttonCount = 11
  buttons = [0,0,0,0,0,0,0,0,0,0,0]
  #
  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui
    
    
  def readData(self, gg, line):
    dataUsed = True
    self.lock.acquire()
    try:
      if gg[0] == 'joy':
        # joy 1 0 8 11 0 -2 -32767 0 -2 -32767 0 0 0 0 0 0 0 0 0 0 0 0 0'
        # 1:  1 = running
        # 2:  0 = not in manual control
        # 3:  8 = number of axis
        # 4: 11 = number of buttons
        # 5,6:  = axis 1,2
        # 7,8:  = axis 3,4
        # 9,10:  = axis 5,6
        # 11,12:  = axis 7,8
        # 13..24: = button 1..11
        if len(gg) > 23:
          self.joyPresent = int(gg[1],0)
          self.manualControl = int(gg[2],0)
          self.axisCount = int(gg[3], 0)
          self.buttonCount = int(gg[4],0)
          for a in range(self.axisCount):
            self.axis[a] = int(gg[a + 5],0)
          for b in range(self.buttonCount):
            self.buttons[b] = int(gg[b + 13],0)
          self.dataRead = True;
          self.gotFirstData = True
        else:
          print("Failed joy message too short {} values need 24!".format(str(len(gg))))
      else:
        dataUsed = False
    except:
      print("Joy: data read error - skipped a " + gg[0] + " from " + line)
      pass
    self.lock.release()
    return dataUsed


  def timerUpdate(self):
    self.lock.acquire()
    self.inTimerUpdate = True
    connected = (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected()
    if self.dataRead:
      self.ui.joy_axis_1.setValue(self.axis[0])
      self.ui.joy_axis_1_num.setText("1: " + str(self.axis[0]))
      self.ui.joy_axis_2.setValue(self.axis[1])
      self.ui.joy_axis_2_num.setText("2: " + str(self.axis[1]))
      self.ui.joy_axis_3.setValue(self.axis[2])
      self.ui.joy_axis_3_num.setText("3: " + str(self.axis[2]))
      self.ui.joy_axis_4.setValue(self.axis[3])
      self.ui.joy_axis_4_num.setText("4: " + str(self.axis[3]))
      self.ui.joy_axis_5.setValue(self.axis[4])
      self.ui.joy_axis_5_num.setText("5: " + str(self.axis[4]))
      self.ui.joy_axis_6.setValue(self.axis[5])
      self.ui.joy_axis_6_num.setText("6: " + str(self.axis[5]))
      self.ui.joy_axis_7.setValue(self.axis[6])
      self.ui.joy_axis_7_num.setText("7: " + str(self.axis[6]))
      self.ui.joy_axis_8.setValue(self.axis[7])
      self.ui.joy_axis_8_num.setText("8: " + str(self.axis[7]))
      self.ui.joy_button_1.setChecked(self.buttons[0])
      self.ui.joy_button_2.setChecked(self.buttons[1])
      self.ui.joy_button_3.setChecked(self.buttons[2])
      self.ui.joy_button_4.setChecked(self.buttons[3])
      self.ui.joy_button_5.setChecked(self.buttons[4])
      self.ui.joy_button_6.setChecked(self.buttons[5])
      self.ui.joy_button_7.setChecked(self.buttons[6])
      self.ui.joy_button_8.setChecked(self.buttons[7])
      self.ui.joy_button_9.setChecked(self.buttons[8])
      self.ui.joy_button_10.setChecked(self.buttons[9])
      self.ui.joy_button_11.setChecked(self.buttons[10])
      if self.manualControl:
        self.ui.joy_manual.setText("Manual")
      else:
        self.ui.joy_manual.setText("Auto (mission)")
      if self.joyPresent:
        self.ui.joy_available.setText("Available")
      else:
        self.ui.joy_available.setText("Not available")
      self.dataRead = False
        #
    self.lock.release()
    #
    if self.robot.currentTab != self.lastTab and self.robot.info.talkToBridge:
      if connected:
        #print("switched tab to " + str(self.robot.currentTab))
        if (self.robot.currentTab == "joy"):
          ##  subscribe to base data IS, version and mission status
          self.robot.conWrite("joy subscribe 6\n")
          self.robot.conWrite("joy get\n")
        if (self.lastTab == "joy"):
          ##  unsubscribe to wifi data
          self.robot.conWrite("joy subscribe 0\n") # old format
        self.lastTab = self.robot.currentTab;
    if (self.gotFirstData and connected):
      # keep flag, until we got data from robot,
      # else we may send default data to robot
      self.inTimerUpdate = False;

  def cancelEdit(self):
    self.inEdit = False;

  def edit(self):
    self.inEdit = True;

  def apply(self):
    #self.applyServo1()
    #self.lastDataRequestTime = time.time()
    self.inEdit = False;

  def justConnected(self):
    self.lastTab = "none"
  
  
