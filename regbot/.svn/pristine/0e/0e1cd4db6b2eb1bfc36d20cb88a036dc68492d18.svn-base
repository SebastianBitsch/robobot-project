#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Data logging functions
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

class UServo(object):
  servoUse = [False, False, False, False, False]
  servoVal = [0,0,0,0,0]
  servoVel = [0,0,0,0,0]
  servo1Steer = False
  steerOffset = 0 # in control units
  steerWheelDist = 0.135 # to steering wheel
  steerScale = 90  # angle change from 1ms to 2 ms
  set_manually = False
  dataRead = False # for all servos
  data1Read = False # for servo 1
  lock = threading.RLock()
  lastDataRequestTime = time.time()
  lastDataSetTime = time.time()
  servoEnableSend = [0, -1, -1, -1, -1, -1]
  servoUseByGUI = [False, False, False, False, False]
  servoUseValue = [0,0,0,0,0]
  inEdit = False
  lastDataRequest = 0
  inFastUpdate = False
  lastTab = ""
  inTimerUpdate = True
  gotFirstData = False;
  #
  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui

  def timerUpdate(self):
    self.lock.acquire()
    self.inTimerUpdate = True
    connected = (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected()
    if self.dataRead:
      if not self.servoUseByGUI[0]:
        self.ui.checkBox_servo1.setChecked(self.servoUse[0])
        self.ui.servo_value_1.setValue(self.servoVal[0])
        self.ui.horizontalSlider_servo1.setValue(self.servoVal[0])
        self.ui.servo1_vel.setValue(self.servoVel[0])
      if not self.servoUseByGUI[1]:
        self.ui.checkBox_servo2.setChecked(self.servoUse[1])
        self.ui.servo_value_2.setValue(self.servoVal[1])
        self.ui.horizontalSlider_servo2.setValue(self.servoVal[1])
        self.ui.servo2_vel.setValue(self.servoVel[1])
      if not self.servoUseByGUI[2]:
        self.ui.checkBox_servo3.setChecked(self.servoUse[2])
        self.ui.servo_value_3.setValue(self.servoVal[2])
        self.ui.horizontalSlider_servo3.setValue(self.servoVal[2])
        self.ui.servo3_vel.setValue(self.servoVel[2])
      if not self.servoUseByGUI[3]:
        self.ui.checkBox_servo4.setChecked(self.servoUse[3])
        self.ui.servo_value_4.setValue(self.servoVal[3])
        self.ui.horizontalSlider_servo4.setValue(self.servoVal[3])
      if not self.servoUseByGUI[4]:
        self.ui.checkBox_servo5.setChecked(self.servoUse[4])
        self.ui.servo_value_5.setValue(self.servoVal[4])
        self.ui.horizontalSlider_servo5.setValue(self.servoVal[4])
      self.dataRead = False
    if (not self.inEdit):
      if (self.data1Read):
        self.data1Read = False
        self.ui.checkBox_servo1_is_steering.setChecked(self.servo1Steer)
        self.ui.val_servo1_offset.setValue(self.steerOffset)
        self.ui.val_steer_distance.setValue(self.steerWheelDist)
        self.ui.val_servo1_scale.setValue(self.steerScale)
        self.gotFirstData = True
      # set buttons not in edit
      self.ui.servo_apply.setEnabled(False)
      self.ui.servo_cancel.setEnabled(False)
      self.ui.servo_edit.setEnabled(True)
    else:
      # set buttons in edit
      self.ui.servo_apply.setEnabled(True)
      self.ui.servo_cancel.setEnabled(True)
      self.ui.servo_edit.setEnabled(False)
        #
    self.lock.release()
    # send servo change - but not too fast 
    if (self.robot.currentTab == "servo"):
      if time.time() - self.lastDataSetTime > 0.1:
        use = [self.ui.checkBox_servo1_use.isChecked(),
              self.ui.checkBox_servo2_use.isChecked(),
              self.ui.checkBox_servo3_use.isChecked(),
              self.ui.checkBox_servo4_use.isChecked(),
              self.ui.checkBox_servo5_use.isChecked()]
        ena = [self.ui.checkBox_servo1.isChecked(),
              self.ui.checkBox_servo2.isChecked(),
              self.ui.checkBox_servo3.isChecked(),
              self.ui.checkBox_servo4.isChecked(),
              self.ui.checkBox_servo5.isChecked()]
        val = [self.ui.servo_value_1.value(),
              self.ui.servo_value_2.value(),
              self.ui.servo_value_3.value(),
              self.ui.servo_value_4.value(),
              self.ui.servo_value_5.value()]
        vel = [self.ui.servo1_vel.value(),
              self.ui.servo2_vel.value(),
              self.ui.servo3_vel.value(),
              0,
              0]
        #print("servo update " + str(ena) + " val " + str(val))
        for i in range(5):
          if use[i]:
            if self.servoEnableSend[i] != ena[i]:
              if not ena[i]:
                s = "servo {} 10000 0\n".format(i+1)
                self.robot.conWrite(s)
              self.servoEnableSend[i] = ena[i]
            if ena[i] and self.servoVal[i] != val[i]:
              s = "servo {} {} {}\n".format(i+1, int(val[i]), int(vel[i]))
              self.robot.conWrite(s)
              self.servoVal[i] = val[i]
        # save check time
        self.lastDataSetTime = time.time()
    #
    # request update at times - if changed by another client
    #if (self.robot.currentTab == "servo"):
      #print("¤ current tab =" + self.robot.currentTab + " wifi waiting " + str(self.robot.wifiWaiting4reply) + " in edit=" + str(self.inEdit))
    if self.robot.currentTab != self.lastTab and self.robot.info.talkToBridge:
      if connected:
        #print("switched tab to " + str(self.robot.currentTab))
        if (self.robot.currentTab == "servo"):
          ##  subscribe to base data IS, version and mission status
          self.robot.conWrite("svs subscribe 6\n")
          self.robot.conWrite("sv0 subscribe 6\n") # old format
          self.robot.conWrite("svo\n")
          self.robot.conWrite("sv1s subscribe 6\n")
          self.robot.conWrite("sv1 subscribe 6\n") # old format
        if (self.lastTab == "servo"):
          ##  unsubscribe to wifi data
          self.robot.conWrite("svo subscribe 0\n") # old format
          self.robot.conWrite("svs subscribe 0\n")
          self.robot.conWrite("sv1 subscribe 0\n") # old format
          self.robot.conWrite("sv1s subscribe 0\n")
        self.lastTab = self.robot.currentTab;
    if connected and self.robot.currentTab == "servo" and not self.inEdit:
      #print("¤ in Servo tab")
      if time.time() - self.lastDataRequestTime > 0.5:
        #print("¤ time for new data")
        if (self.lastDataRequest == 2):
          self.lastDataRequest = 1
          self.robot.conWrite("sv1\n")
        else:
          self.lastDataRequest = 2
          self.robot.conWrite("svo\n")
        self.lastDataRequestTime = time.time()
    if (self.gotFirstData and connected):
      # keep flag, until we got data from robot,
      # else we may send default data to robot
      self.inTimerUpdate = False;

  def cancelEdit(self):
    self.inEdit = False;

  def edit(self):
    self.inEdit = True;

  def apply(self):
    #self.applyServo()
    self.applyServo1()
    self.lastDataRequestTime = time.time()
    self.inEdit = False;

  def applyServo1(self):
    # set steering parameters
    if (self.gotFirstData):
      s = "sv1 {} {} {} {}\n".format(
        int(self.ui.checkBox_servo1_is_steering.isChecked()),
        int(self.ui.val_servo1_offset.value()),
        self.ui.val_steer_distance.value(),
        self.ui.val_servo1_scale.value()
        )
      self.robot.conWrite(s)
    pass

  #def applyServo(self):
    ## last parameter is servo velocity - disabled to '0'
    #s = "svo {} {} 0 {} {} 0 {} {} 0 {} {} 0 {} {} 0\n".format(
      #int(self.ui.checkBox_servo1.isChecked()),
      #self.ui.servo_value_1.value(),
      #int(self.ui.checkBox_servo2.isChecked()),
      #self.ui.servo_value_2.value(),
      #int(self.ui.checkBox_servo3.isChecked()),
      #self.ui.servo_value_3.value(),
      #int(self.ui.checkBox_servo4.isChecked()),
      #self.ui.servo_value_4.value(),
      #int(self.ui.checkBox_servo5.isChecked()),
      #self.ui.servo_value_5.value()
      #)
    #self.robot.conWrite(s)
    #pass
  
  def setSingleServo(self, idx, enable, value):
    if time.time() - self.lastDataSetTime > 0.1:
      if enable:
        #s = "servo {} {}\n".format(idx, int(value))
        #self.robot.conWrite(s)
        #self.lastDataSetTime = time.time()
        self.servoEnableSend[idx] = enable
        self.servoVal[idx] = value
      elif self.servoEnableSend[idx] != enable:
        # disable
        #s = "servo {} 10000\n".format(idx)
        #self.robot.conWrite(s)
        self.servoEnableSend[idx] = enable
        self.servoVal[idx] = 10000
    pass
  #
  def readData(self, gg, line):
    dataUsed = True
    self.lock.acquire()
    try:
      if gg[0] == 'sv1' or gg[0] == 'sv1s' :
        if len(gg) > 3:
          self.servo1Steer = int(gg[1],0)
          self.steerOffset = int(gg[2],0)
          self.steerWheelDist = float(gg[3])
          self.steerScale = float(gg[4])
          self.data1Read = True;
          #print("UServo: got sv1 line {}\r\n".format(line))
          #print("UServo: got sv1 parameters {} {} {} {}\r\n".format(self.servo1Steer, self.steerOffset, self.steerWheelDist, self.steerScale))
      elif gg[0] == "svo" or gg[0] == "svs":
        # the svo message is renamed to svs as the same name
        # should not be used both ways - the bridge don't is confused otherwise
        if len(gg) > 11 and len(gg) <= 15:
          self.servoUse[0] = int(gg[1],0)
          self.servoVal[0] = int(gg[2],0)
          self.servoUse[1] = int(gg[3],0)
          self.servoVal[1] = int(gg[4],0)
          self.servoUse[2] = int(gg[5],0)
          self.servoVal[2] = int(gg[6],0)
          self.servoUse[3] = int(gg[7],0)
          self.servoVal[3] = int(gg[8],0)
          self.servoUse[4] = int(gg[9],0)
          self.servoVal[4] = int(gg[10],0)
          self.servo1Steer = int(gg[11],0)
          self.steerOffset = int(gg[12],0)
          self.steerWheelDist = float(gg[13])
          self.steerScale = float(gg[14])
          self.data1Read = True
        elif len(gg) > 17:
          # new version with velocity
          self.servoUse[0] = int(gg[1],0)
          self.servoVal[0] = int(gg[2],0)
          self.servoVel[0] = int(gg[3],0)
          #
          self.servoUse[1] = int(gg[4],0)
          self.servoVal[1] = int(gg[5],0)
          self.servoVel[1] = int(gg[6],0)
          #
          self.servoUse[2] = int(gg[7],0)
          self.servoVal[2] = int(gg[8],0)
          self.servoVel[2] = int(gg[9],0)
          #
          self.servoUse[3] = int(gg[10],0)
          self.servoVal[3] = int(gg[11],0)
          self.servoUse[4] = int(gg[12],0)
          self.servoVal[4] = int(gg[13],0)
          self.servo1Steer = int(gg[14],0)
          self.steerOffset = int(gg[15],0)
          self.steerWheelDist = float(gg[16])
          self.steerScale = float(gg[17])
          self.data1Read = True
        elif len(gg) > 5:
          print("Failed svo message too short {} values!".format(str(len(gg))))
      else:
        dataUsed = False
        #self.dataRead = True
    except:
      print("UServo: data read error - skipped a " + gg[0] + " from " + line)
      pass
    self.lock.release()
    return dataUsed
  #def logSetManually(self):
    #if (not self.log_set_manually):
      #self.log_set_manually = True
      #self.ui.log_flag_apply.setEnabled(True)
  def logSave(self):
    try:
      f = open(self.ui.log_filename.text(), "w")
      if (self.ui.log_save_header.isChecked()):
        f.write('%% logfile from robot ' + self.ui.robot_id_main.text() + '\n')
        f.write(self.logList)
      if (self.ui.log_save_config.isChecked()):
        f.write('%% Configuration as seen in client (assumed updated from robot)\n')
        fil = open('.regbotConfigTemp.ini', 'r');
        for line in fil:
          f.write('% ' + line)
        fil.close()
        f.write('%% data log\n')
      f.write(self.logData)
      f.close()
    except:
      self.ui.statusbar.showMessage("Failed to open file " + self.ui.log_filename.text() + "!", 3000)
  def logClear(self):
    self.logData = ""
    self.logList = ""
    self.logDataRead = True

  def servo1bar(self):
    if not self.inFastUpdate and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.servo_value_1.setValue(self.ui.horizontalSlider_servo1.value())
      #self.setSingleServo(1, self.ui.checkBox_servo1.isChecked(), self.ui.horizontalSlider_servo1.value())
      self.inFastUpdate = False
    pass      

  def servo2bar(self):
    if not self.inFastUpdate and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.servo_value_2.setValue(self.ui.horizontalSlider_servo2.value())
      #self.setSingleServo(2, self.ui.checkBox_servo2.isChecked(), self.ui.horizontalSlider_servo2.value())
      self.inFastUpdate = False
    pass      

  def servo3bar(self):
    if not self.inFastUpdate and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.servo_value_3.setValue(self.ui.horizontalSlider_servo3.value())
      #self.setSingleServo(3, self.ui.checkBox_servo3.isChecked(), self.ui.horizontalSlider_servo3.value())
      self.inFastUpdate = False
    pass      

  def servo4bar(self):
    if not self.inFastUpdate and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.servo_value_4.setValue(self.ui.horizontalSlider_servo4.value())
      #self.setSingleServo(4, self.ui.checkBox_servo4.isChecked(), self.ui.horizontalSlider_servo4.value())
      self.inFastUpdate = False
    pass      

  def servo5bar(self):
    if not self.inFastUpdate and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.servo_value_5.setValue(self.ui.horizontalSlider_servo5.value())
      #self.setSingleServo(5, self.ui.checkBox_servo5.isChecked(), self.ui.horizontalSlider_servo5.value())
      self.inFastUpdate = False
    pass      

  def servo1num(self):
    if not self.inFastUpdate  and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.horizontalSlider_servo1.setValue(self.ui.servo_value_1.value())
      #self.setSingleServo(1, self.ui.checkBox_servo1.isChecked(), self.ui.horizontalSlider_servo1.value())
      self.inFastUpdate = False
    pass      

  def servo2num(self):
    if not self.inFastUpdate  and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.horizontalSlider_servo2.setValue(self.ui.servo_value_2.value())
      #self.setSingleServo(2, self.ui.checkBox_servo2.isChecked(), self.ui.horizontalSlider_servo2.value())
      self.inFastUpdate = False
    pass      
      
  def servo3num(self):
    if not self.inFastUpdate  and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.horizontalSlider_servo3.setValue(self.ui.servo_value_3.value())
      #self.setSingleServo(3, self.ui.checkBox_servo3.isChecked(), self.ui.horizontalSlider_servo3.value())
      self.inFastUpdate = False
    pass

  def servo4num(self):
    if not self.inFastUpdate and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.horizontalSlider_servo4.setValue(self.ui.servo_value_4.value())
      #self.setSingleServo(4, self.ui.checkBox_servo4.isChecked(), self.ui.horizontalSlider_servo4.value())
      self.inFastUpdate = False
    pass      

  def servo5num(self):
    if not self.inFastUpdate  and not self.inTimerUpdate:
      #self.inEdit = True;
      self.inFastUpdate = True
      self.ui.horizontalSlider_servo5.setValue(self.ui.servo_value_5.value())
      #self.setSingleServo(5, self.ui.checkBox_servo5.isChecked(), self.ui.horizontalSlider_servo5.value())
      self.inFastUpdate = False
    pass      
  
  def servo1use(self):
    self.servoUseByGUI[0] = self.ui.checkBox_servo1_use.isChecked()
    self.ui.checkBox_servo1.setEnabled(self.servoUseByGUI[0])
    self.ui.servo_value_1.setEnabled(self.servoUseByGUI[0])
    self.ui.horizontalSlider_servo1.setEnabled(self.servoUseByGUI[0])
    self.ui.servo1_vel.setEnabled(self.servoUseByGUI[0])
    pass      

  def servo2use(self):
    self.servoUseByGUI[1] = self.ui.checkBox_servo2_use.isChecked()
    self.ui.checkBox_servo2.setEnabled(self.servoUseByGUI[1])
    self.ui.servo_value_2.setEnabled(self.servoUseByGUI[1])
    self.ui.horizontalSlider_servo2.setEnabled(self.servoUseByGUI[1])
    self.ui.servo2_vel.setEnabled(self.servoUseByGUI[1])
    pass      

  def servo3use(self):
    self.servoUseByGUI[2] = self.ui.checkBox_servo3_use.isChecked()
    self.ui.checkBox_servo3.setEnabled(self.servoUseByGUI[2])
    self.ui.servo_value_3.setEnabled(self.servoUseByGUI[2])
    self.ui.horizontalSlider_servo3.setEnabled(self.servoUseByGUI[2])
    self.ui.servo3_vel.setEnabled(self.servoUseByGUI[2])
    pass      

  def servo4use(self):
    self.servoUseByGUI[3] = self.ui.checkBox_servo4_use.isChecked()
    self.ui.checkBox_servo4.setEnabled(self.servoUseByGUI[3])
    self.ui.servo_value_4.setEnabled(self.servoUseByGUI[3])
    self.ui.horizontalSlider_servo4.setEnabled(self.servoUseByGUI[3])
    pass      

  def servo5use(self):
    self.servoUseByGUI[4] = self.ui.checkBox_servo5_use.isChecked()
    self.ui.checkBox_servo5.setEnabled(self.servoUseByGUI[4])
    self.ui.servo_value_5.setEnabled(self.servoUseByGUI[4])
    self.ui.horizontalSlider_servo5.setEnabled(self.servoUseByGUI[4])
    pass      

  def justConnected(self):
    self.lastTab = "none"
