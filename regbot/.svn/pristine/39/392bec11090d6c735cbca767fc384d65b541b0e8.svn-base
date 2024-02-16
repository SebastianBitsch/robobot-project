#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Mostly robot specific info, but also wifi settings
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
#import pyqtgraph as pg

 

class URobotInfo(object):
  wheelbase = 0.0
  gear = 0.0
  pulsePerRev = 0
  wheelLRadius = 0.0
  wheelRRadius = 0.0
  version = ""
  robotID = 0
  robotHWtype = 2 # type 1 is old with no wifi and line sensor, 2 if with satellite PCB for wifi and power, 3 is with power mgmt on board
  balanceOffset = 0.0
  batteryUse = True
  reverseMotor = False
  reverseEncoderLeft = False
  reverseEncoderRight = False
  batteryIdleVolt = 9.9
  name = "empty"
  gotRobotName = False
  wifiIP = "0.0.0.0"
  wifiMAC= "00:00:00:00:00:00"
  #wifiport = 24001

class UInfo(object):
  dataRead = True
  dataWifi = True
  dataClient = True
  robotID = 0
  gyroOK = False
  lock = threading.RLock()
  robots = [URobotInfo]
  thisRobot = robots[0]
  inEdit = False
  wifiUse = False
  wifiSSID = "aaa"
  wifiPW = ""
  wifiGotIP = False
  wifiPortOpen = False
  wifiInEdit = False
  wifiSetupState = -1;
  wifiSleep = False;
  wifiPort = 24001
  clientRxCnt = [0,0,0,0,0]
  clientTxCnt = [0,0,0,0,0]
  wifiGood = 0
  #wifiLost = 0
  wifiLoss = 0.0
  lastDataRequestTime = time.time()
  lastDataRequest = 0
  lastAliveTime = 0.0
  talkToBridge = False
  talkToBridgeOld = False;
  lastTab = ""
  regbotSampleTime = 0.001
  dataReadTiming = False;
  #  
  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui

  def gotAliveMsg(self):
    self.lastAliveTime = time.time()
  # find robot with this ID, and create it if it is not there
  def getRobot(self, id):
    rb = []
    for rb in self.robots:
      if rb.robotID == id:
        #print("found robot " + str(id))
        return rb
    rb = URobotInfo()
    rb.robotID = id
    self.robots.append(rb)
    #print("made robot with id " + str(id))
    return rb
  #
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "rid":
        self.robotID = int(gg[1],10)
        self.thisRobot = self.getRobot(self.robotID)
        self.thisRobot.wheelbase = float(gg[2])
        self.thisRobot.gear = float(gg[3])
        self.thisRobot.pulsePerRev = int(gg[4], 10)
        self.thisRobot.wheelLRadius = float(gg[5])
        self.thisRobot.wheelRRadius = float(gg[6])
        self.thisRobot.balanceOffset = float(gg[7])
        flags = int(gg[8])
        self.thisRobot.batteryUse = flags & 1
        self.thisRobot.reverseMotor = flags & 2
        self.thisRobot.reverseEncoderLeft = flags & 4
        self.thisRobot.reverseEncoderRight = flags & 8
        self.thisRobot.batteryIdleVolt = float(gg[9])
        try:
          self.thisRobot.robotHWtype = float(gg[10])
          self.thisRobot.name = gg[11]
        except:
          # missing robot hardware version
          self.thisRobot.robotHWtype = 2;
          self.thisRobot.name = gg[10]
        self.thisRobot.gotRobotName = True
        self.dataRead = True
      #
      elif gg[0] == "version":
        self.thisRobot.version = gg[1]
        self.gyroOK = int(gg[2],10)
        self.dataRead = True
      elif gg[0] == "vti": # static timing info (request using "v4")
        if len(gg) > 1:
          self.regbotSampleTime = float(gg[1])
          self.dataReadTiming = True
          pass
      # wifi info
      elif gg[0] == "wfi":
        self.wifiUse = bool(gg[1] == "1")
        self.wifiSetupState = int(gg[2])
        # debug
        #print("# got wfi use = " + str(self.wifiUse) + " (" + gg[1] + "), setup=" + gg[2] + ", sleep=" + gg[4])
        ## debug end
        self.wifiPortOpen = gg[2] == "99"
        self.wifiGotIP = gg[3] >= "3"
        self.wifiSleep = bool(gg[4]=="1")
        self.thisRobot.wifiIP = gg[5] + '.' + gg[6] + '.' + gg[7] + '.' + gg[8]
        # got also MAC
        self.thisRobot.wifiMAC = gg[9]
        self.wifiPort = int(gg[10], 0)
        self.wifiSSID = "no data"
        if (len(gg) > 11):
          # get number of characters in password
          n = int(gg[11], 10)
          #print("wifi pw length=" + str(n))
          if n < 32:
            self.wifiPW = ""
            for i in range(1,n):
              self.wifiPW += "*"
          else:
            print("wifi error in password length >=32")
          #print("wifi pw =" + self.wifiPW)
        if (len(gg) > 12):
          #print("wifi got SSID=" + gg[11] + " len(gg)=" + str(len(gg)))
          self.wifiSSID = ""
          for i in range(12,len(gg)):
            self.wifiSSID += gg[i] + " "
          #print("wifi SSID =" + self.wifiSSID)
        else:
          print("wifi no SSID")
        self.dataWifi = True
        #print("# got wfi")
      # client info
      elif gg[0] == "wfc":
        self.clientRxCnt[0] = int(gg[1], 10)
        self.clientTxCnt[0] = int(gg[2], 10)
        self.clientRxCnt[1] = int(gg[3], 10)
        self.clientTxCnt[1] = int(gg[4], 10)
        self.clientRxCnt[2] = int(gg[5], 10)
        self.clientTxCnt[2] = int(gg[6], 10)
        self.clientRxCnt[3] = int(gg[7], 10)
        self.clientTxCnt[3] = int(gg[8], 10)
        self.clientRxCnt[4] = int(gg[9], 10)
        self.clientTxCnt[4] = int(gg[10], 10)
        #self.wifiGood = int(gg[11], 10)
        #self.wifiLost = int(gg[12], 10)
        self.dataClient = True
      elif gg[0] == "bridge":
        self.talkToBridge = True
        print("set talkToBridge=True")
      else:
        used = False
    except:
      print("URobot: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  #
  def justConnected(self):
    # get robot ID
    self.robot.conWrite("u4\n")
    # and version number
    self.robot.conWrite("u0\n")
    # and static timing
    self.robot.conWrite("v4\n")
    self.lastTab = "none"
    print("# Info justConnected() called")
  #
  def timerUpdate(self):
    if (self.talkToBridge != self.talkToBridgeOld):
      self.talkToBridgeOld = self.talkToBridge
      if (self.talkToBridge):
        self.ui.connect_wifi.setText("Network (bridge)")
        # initiate bas subscriptions
        self.robot.conWrite("# subscribe 6\n")
        self.robot.conWrite("logdata subscribe 6\n")
        self.robot.conWrite("hbt subscribe 3\n")
      else:
        self.ui.connect_wifi.setText("Network")
    if (self.dataRead):
      self.dataRead = False
      self.lock.acquire()
      if (not self.inEdit):
        # set new values
        self.ui.robot_gear.setProperty("value", self.thisRobot.gear)
        self.ui.robot_pulse_per_rev.setValue(self.thisRobot.pulsePerRev)
        self.ui.robot_wheel_radius_left.setValue(self.thisRobot.wheelLRadius)
        self.ui.robot_wheel_radius_right.setValue(self.thisRobot.wheelRRadius)
        self.ui.robot_hw_type.setValue(self.thisRobot.robotHWtype)
        self.ui.robot_balance_offset.setValue(self.thisRobot.balanceOffset)
        self.ui.save_id_on_robot.setEnabled(False)
        self.ui.robot_base.setValue(self.thisRobot.wheelbase)
        self.ui.robot_id.setValue(self.robotID)
        self.ui.robot_id_main.setText(self.thisRobot.name + " (" + str(self.robotID) + ")")
        self.ui.robot_on_battery.setChecked(self.thisRobot.batteryUse)
        self.ui.robot_rev_motor.setChecked(self.thisRobot.reverseMotor)
        self.ui.robot_rev_encoder.setChecked(self.thisRobot.reverseEncoderLeft)
        self.ui.robot_rev_encoder_right.setChecked(self.thisRobot.reverseEncoderRight)
        self.ui.robot_battery_idle_volt.setValue(self.thisRobot.batteryIdleVolt)
      self.lock.release()
    if (self.dataReadTiming):
      # static timing
      self.dataReadTiming = False
      # set new values
      self.ui.regbotSampleTime.setProperty("value", self.regbotSampleTime)
    if (not self.inEdit):
      # and buttons
      self.ui.save_id_on_robot.setEnabled(False)
      self.ui.robot_cancel.setEnabled(False)
      self.ui.robot_edit.setEnabled(True)
    else:
      self.ui.save_id_on_robot.setEnabled(True)
      self.ui.robot_cancel.setEnabled(True)
      self.ui.robot_edit.setEnabled(False)
    # wifi alive
    if time.time() - self.lastAliveTime > 5:
      self.ui.wifi_sendAlive.setChecked(False)
    else:
      self.ui.wifi_sendAlive.setChecked(True)
    if (self.dataWifi):
      #print("# showing wfi")
      self.ui.wifi_got_ip.setChecked(self.wifiGotIP)
      self.ui.wifi_port_open.setChecked(self.wifiPortOpen)
      self.ui.wifi_ip.setText(self.thisRobot.wifiIP)
      self.ui.wifi_mac.setText(self.thisRobot.wifiMAC)
      if (not self.wifiInEdit):
        # update fields with data received from robot
        #print("wifi update to" + self.wifiSSID + " " + self.wifiPW)
        self.lock.acquire()
        self.ui.wifi_use.setChecked(self.wifiUse and not self.wifiSleep)
        #print("# wfi use setChecked = " + str(self.wifiUse))
        self.ui.wifi_port.setText(str(self.wifiPort))
        self.ui.wifi_ssid.setText(self.wifiSSID)
        self.ui.wifi_pw.setText(self.wifiPW)
        # set button properties
        self.ui.wifi_use.setEnabled(False)
        self.ui.wifi_port.setReadOnly(True)
        self.ui.wifi_ssid.setReadOnly(True)
        self.ui.wifi_pw.setReadOnly(True)
        self.ui.wifi_ip.setReadOnly(True)
        self.ui.wifi_apply.setEnabled(False)
        self.ui.wifi_got_ip.setEnabled(True)
        self.ui.wifi_port_open.setEnabled(True)
        self.lock.release()
      else:
        self.ui.wifi_use.setEnabled(True)
        self.ui.wifi_port.setReadOnly(False)
        self.ui.wifi_ssid.setReadOnly(False)
        self.ui.wifi_pw.setReadOnly(False)
        #print("wifi in edit")
      # data implemented / or potentially too old
      self.dataWifi = False
    # reqest data periodically, as these data may change
    if self.robot.currentTab != self.lastTab and self.talkToBridge:
      if (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected():
        print("switched tab to " + str(self.robot.currentTab))
        if (self.robot.currentTab == "robot"):
          ##  subscribe to base data IS, version and mission status
          self.robot.conWrite("rid subscribe 6\n")
          self.robot.conWrite("u4\n")
          self.robot.conWrite("version subscribe 6\n")
          self.robot.conWrite("u0\n")
          self.robot.conWrite("mis subscribe 6\n")
          self.robot.conWrite("u2\n")
          self.robot.conWrite("vti subscribe 6\n") # static timing info
          self.robot.conWrite("v4\n")
        elif (self.robot.currentTab == "wifi"):
          ##  subscribe to base wifi data
          self.robot.conWrite("wfi subscribe 2\n")
          self.robot.conWrite("wfc subscribe 2\n")
        if (self.lastTab == "wifi"):
          ##  unsubscribe to wifi data
          self.robot.conWrite("wfi subscribe 0\n")
          self.robot.conWrite("wfc subscribe 0\n")
        self.lastTab = self.robot.currentTab;
    # update request    
    if self.robot.currentTab == "wifi":
      # as data takes time to change - request more data regularly
      if time.time() - self.lastDataRequestTime > 0.5:
        if (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected():
          if (self.lastDataRequest == 2):
            self.lastDataRequest = 1
            self.robot.conWrite("v1\n")
          else:
            self.lastDataRequest = 2
            self.robot.conWrite("v2\n")
        elif self.robot.wifiWaiting4reply:
          print("wifi tab (uinfo.py): waiting for reply when want to send data request")
        self.lastDataRequestTime = time.time()
      self.ui.wifi_connection_tx.setText(str(self.robot.wifiTxCnt))
      self.ui.wifi_connection_rx.setText(str(self.robot.wifiRxCnt))
    # info about wifi clients
    if self.dataClient:
      self.lock.acquire()
      self.ui.wifi_client_1.setText(str(self.clientTxCnt[0]))
      self.ui.wifi_client_2.setText(str(self.clientTxCnt[1]))
      self.ui.wifi_client_3.setText(str(self.clientTxCnt[2]))
      self.ui.wifi_client_4.setText(str(self.clientTxCnt[3]))
      self.ui.wifi_client_5.setText(str(self.clientTxCnt[4]))
      self.ui.wifi_client_1_rx.setText(str(self.clientRxCnt[0]))
      self.ui.wifi_client_2_rx.setText(str(self.clientRxCnt[1]))
      self.ui.wifi_client_3_rx.setText(str(self.clientRxCnt[2]))
      self.ui.wifi_client_4_rx.setText(str(self.clientRxCnt[3]))
      self.ui.wifi_client_5_rx.setText(str(self.clientRxCnt[4]))
      self.dataClient = False
      self.lock.release()
    pass
  # 
  def dataChangedManually(self):
    # robot static parameters
    if (not self.robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      self.lock.release()
  #def dataChangedManuallyWifi(self):
    #if (not self.robot.timerUpdate):
      #self.lock.acquire()
      #self.wifiInEdit = True
      #self.lock.release()
  #
  def cancelEdit(self):
    self.inEdit = False;
  # send data as is in edit fields
  def wifiSendData(self):
    s = ('wifi ' + str(int(self.ui.wifi_use.isChecked())) + ' ' + str(self.ui.wifi_port.text()) + 
              ' "' + str(self.ui.wifi_ssid.text()).strip() + '" "' + 
                     str(self.ui.wifi_pw.text()).strip() + '"\n')
    self.robot.conWrite(s)
    self.wifiInEdit = False
    self.ui.wifi_edit.setEnabled(True)
    self.ui.wifi_cancel.setEnabled(False)
    self.ui.wifi_apply.setEnabled(False)
    pass
  # Cansel wifi edit
  def wifiCancel(self):
    #self.robot.conWrite("v1\n")
    print("cancel wifi edit")
    self.wifiInEdit = False
    self.ui.wifi_port.setReadOnly(True)
    self.ui.wifi_ssid.setReadOnly(True)
    self.ui.wifi_pw.setReadOnly(True)
    self.ui.wifi_use.setEnabled(False)
    self.ui.wifi_edit.setEnabled(True)
    self.ui.wifi_cancel.setEnabled(False)
    self.ui.wifi_apply.setEnabled(False)
    pass
  # wifi fields edit
  def wifiEdit(self):
    self.lock.acquire()
    self.wifiInEdit = True
    self.ui.wifi_port.setReadOnly(False)
    self.ui.wifi_ssid.setReadOnly(False)
    self.ui.wifi_pw.setReadOnly(False)
    self.ui.wifi_use.setEnabled(True)
    self.ui.wifi_edit.setEnabled(False)
    self.ui.wifi_cancel.setEnabled(True)
    self.ui.wifi_apply.setEnabled(True)
    self.lock.release()
    print("wifi going in edit mode")
    pass
  def wifiSaveMacList(self):
    try:
      fn = "regbot_mac.txt"
      f = open(fn, "w")
      f.write('%% MAC list for robots in regbot.ini file\r\n')
      for rb in self.robot.info.robots:
        if (rb.robotID == self.robotID):
          # current robot
          if (self.ui.wifi_and_save_IP.isChecked()):
            f.write(str(self.thisRobot.wifiMAC) + str(" 10.16.166." + str(self.thisRobot.robotID) + ' "(' + self.thisRobot.wifiIP + " " + self.thisRobot.name) + ')"\r\n')
          else:
            f.write(str(self.thisRobot.wifiMAC) + str(" 10.16.166." + str(self.thisRobot.robotID) + ' "(' + self.thisRobot.name) + ')"\r\n')
        elif rb.robotID > 0:
          if (self.ui.wifi_and_save_IP.isChecked()):
            f.write(str(rb.wifiMAC) +  " 10.16.166." + str(rb.robotID) + ' "(' + str(rb.wifiIP) + " " + str(rb.name) + ')"\r\n')
          else:
            f.write(str(rb.wifiMAC) +  " 10.16.166." + str(rb.robotID) + ' "(' + str(rb.name) + ')"\r\n')
      f.close()
    except:
      self.ui.statusbar.showMessage("Failed to open file " + fn + "!", 3000)
    pass

