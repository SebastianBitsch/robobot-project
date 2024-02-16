#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* IMU functions
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
import numpy as np
import pyqtgraph as pg
import time


class UEncoder(object):
  "Class to handle IMU data"
  motor1 = []
  motor2 = []
  calibrated = False
  useCalibration = False
  collectData = True
  calibrateAuto = False
  calibrate_index = -1
  encoderTics = 48 # per revolution
  f_cpu = 920000000 # CPU clock frequency (timing)
  lock = threading.RLock()
  encoderDataM0read = False
  encoderDataM1read = False
  encoderFacM0read = False;
  encoderFacM1read = False
  encoderStatusRead = False
  enc1string = "m1 no data"
  enc2string = "m2 no data"
  encFac1string = "m1 no fac data"
  encFac2string = "m2 no fac data"
  # plot if IMU data - test
  epw = 0 # handle for plot window
  epd = 0 # plot data handle
  idx = 0
  data = np.zeros((4,encoderTics))
  dataFac = np.zeros((4,encoderTics))
  lastDataRequestTime = time.time()
  lastDataRequest = 0
  lastTab = ""
  waitingForData = False
  # is motor voltage set for calibration?
  motorVoltageSet = False
  # first focus is used to set checkboxes
  firstFocus = True
  lastTab = ""
  #datat = np.zeros(100)
  def __init__(self, robot, ui):
    self.robot = robot
    self.ui = ui
  #
  def initGraph(self):
    "initialize graph plot, IMU data"
    self.epw = pg.PlotWidget(name='Encoder-timing',title='Encoder timing')  ## giving the plots names allows us to link their axes together
    #self.pwg.setWindowTitle('IMU gyro')
    self.epw.setLabel('left','tick time','s')
    self.epw.setLabel('bottom','encoder tick')
    self.epw.addLegend()    
    self.ui.encoder_graph.addWidget(self.epw)
    self.epd = [self.epw.plot(pen='r',name='motor 1'), self.epw.plot(pen='b',name='motor 2'),
                self.epw.plot(pen='y',name='motor 1 calibrated'), self.epw.plot(pen='c',name='motor 2 calibrated')]
    self.epd[0].setData(self.data[0])
    self.epd[1].setData(self.data[1])
    self.epd[2].setData(self.data[2])
    self.epd[3].setData(self.data[3])
  #
  # data received in read thread
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "en0":
        self.encoderTics = int(gg[1])
        self.f_cpu = int(gg[2])
        self.enc1string = gg[0] + " " + gg[1] + " " + gg[2]
        for i in range(0, self.encoderTics):
          self.data[0,i] = float(gg[i+3])/self.f_cpu
          self.enc1string += " " + gg[i+3]
        self.encoderDataM0read = True
        self.waitingForData = False
      elif gg[0] == "en1":
        self.encoderTics = int(gg[1])
        self.f_cpu = int(gg[2])
        self.enc2string = gg[0] + " " + gg[1] + " " + gg[2]
        for i in range(0, self.encoderTics):
          self.data[1,i] = float(gg[i+3])/self.f_cpu
          self.enc2string += " " + gg[i+3]
        self.encoderDataM1read = True
        self.waitingForData = False
      elif gg[0] == "ef0":
        self.encoderTics = int(gg[1])
        self.encFac1string = gg[0] + " " + gg[1]
        for i in range(0, self.encoderTics):
          self.dataFac[0,i] = float(gg[i+2])
          self.data[2,i] = self.data[0,i] * self.dataFac[0,i]
          self.encFac1string += " " + gg[i+2]
        self.encoderFacM0read = True
        self.waitingForData = False
      elif gg[0] == "ef1":
        self.encoderTics = int(gg[1])
        self.encFac2string = gg[0] + " " + gg[1]
        for i in range(0, self.encoderTics):
          self.dataFac[1,i] = float(gg[i+2])
          self.data[3,i] = self.data[1,i] * self.dataFac[1,i]
          self.encFac2string += " " + gg[i+2]
        self.encoderFacM1read = True
        self.waitingForData = False
      elif gg[0] == "ecs":
        self.calibrated = gg[1] == "1"
        self.calibrateAuto = gg[2] == "1"
        self.collectData = gg[3] == "1"
        self.useCalibration = gg[4] == "1"
        self.calibrate_index = int(gg[5])
        self.encoderStatusRead = True
        self.waitingForData = False
        #print("got encoder status, auto " + gg[2] + "=" + str(self.calibrateAuto))
      else:
        used = False
    except:
      print("UEncoder: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  #
  # time to update GUI
  def timerUpdate(self):
    #gotReply = self.encoderDataM0read  or self.encoderDataM1read or self.encoderFacM0read or self.encoderFacM1read or self.encoderStatusRead
    if self.encoderDataM0read:
      self.encoderDataM0read = False
      #self.ui.val_acc.setValue(self.acc[0])
      #self.ui.val_acc_2.setValue(self.acc[1])
      #self.ui.val_acc_3.setValue(self.acc[2])
      self.epd[0].setData(self.data[0])
      #self.epd[1].setData(self.data[1])
      self.ui.encoder_string.setText(self.enc1string + "\r\n" + self.enc2string + "\r\n" + self.encFac1string + "\r\n" + self.encFac2string)
    if self.encoderDataM1read:
      self.encoderDataM1read = False
      #self.ui.val_acc.setValue(self.acc[0])
      #self.ui.val_acc_2.setValue(self.acc[1])
      #self.ui.val_acc_3.setValue(self.acc[2])
      #self.epd[0].setData(self.data[0])
      self.epd[1].setData(self.data[1])
      self.ui.encoder_string.setText(self.enc1string + "\r\n" + self.enc2string + "\r\n" + self.encFac1string + "\r\n" + self.encFac2string)
    if self.encoderFacM0read:
      self.encoderFacM0read = False
      self.epd[2].setData(self.data[2])
      self.ui.encoder_string.setText(self.enc1string + "\r\n" + self.enc2string + "\r\n" + self.encFac1string + "\r\n" + self.encFac2string)
    if self.encoderFacM1read:
      self.encoderFacM1read = False
      self.epd[3].setData(self.data[3])
      self.ui.encoder_string.setText(self.enc1string + "\r\n" + self.enc2string + "\r\n" + self.encFac1string + "\r\n" + self.encFac2string)
    if self.encoderStatusRead:
      self.encoderStatusRead = False
      self.ui.encoder_is_calibrated.setChecked(self.calibrated)
      #print("is calibrated " + str(self.calibrated))
      self.ui.encoder_collecting.setChecked(self.collectData)
      self.ui.encoder_using.setChecked(self.useCalibration)
      self.ui.encoder_auto.setChecked(self.calibrateAuto)
      self.ui.encoder_index.setText(str(self.calibrate_index))
      if self.firstFocus:
        # set main checkboxes as current status
        self.ui.encoder_collect.setChecked(self.collectData)
        self.ui.encoder_use_calibration.setChecked(self.useCalibration)
        self.ui.encoder_auto_calibrate.setChecked(self.calibrateAuto)
        self.firstFocus=False
        print("setting also checkboxes")
        pass
    # tilt set from drive.py
    #self.cgt.setData(self.datat)
    if self.robot.info.talkToBridge:
      if self.lastTab != self.robot.currentTab:
        #if self.robot.wifiConnected or self.robot.isConnected:
        if self.robot.currentTab == "encoder":
          self.robot.conWrite("en0 subscribe 2\n")
          self.robot.conWrite("en1 subscribe 2\n")
          self.robot.conWrite("ef0 subscribe 2\n")
          self.robot.conWrite("ef1 subscribe 2\n")
          self.robot.conWrite("ecs subscribe 3\n")
        elif (self.lastTab == "encoder"):
          # unsubscribe
          self.robot.conWrite("en0 subscribe 0\n")
          self.robot.conWrite("en1 subscribe 0\n")
          self.robot.conWrite("ef0 subscribe 0\n")
          self.robot.conWrite("ef1 subscribe 0\n")
          self.robot.conWrite("ecs subscribe 0\n")
      self.lastTab = self.robot.currentTab
    if self.robot.currentTab == "encoder":    
      # as data takes time to change - request more data regularly
      if self.robot.wifiConnected or self.robot.isConnected:
        dt = time.time() - self.lastDataRequestTime
        if not self.waitingForData and dt > 0.01:
          self.lastDataRequestTime = time.time()
          self.waitingForData = True
          if self.lastDataRequest == 0:
            self.robot.conWrite("u29\n") # get generel calibrate status
          elif self.lastDataRequest == 1:
            self.robot.conWrite("u19\n") # get encoder tick count
            self.waitingForData = False
          if self.ui.encoder_get_raw_data.isChecked() and self.lastDataRequest > 1:
            if (self.lastDataRequest == 2):
              self.robot.conWrite("u26\n") # motor 2 raw data
            elif self.lastDataRequest == 3:
              self.robot.conWrite("u27\n") # calibrate factor motor 1
            elif self.lastDataRequest == 4:
              self.robot.conWrite("u28\n") # calibrate factor motor 2
            elif self.lastDataRequest == 5:
              self.robot.conWrite("u25\n") # motor 1 raw data
            else:
              self.lastDataRequest = -1
              self.waitingForData = False
          elif not self.ui.encoder_get_raw_data.isChecked() and self.lastDataRequest > 1:
            self.lastDataRequest = -1
          self.lastDataRequest += 1
        elif (dt > 1.5):
          # timeout
          self.waitingForData = False
          #print("Encoder timeout (LDR=" + str(self.lastDataRequest) + ")")
        pass
      pass
    else:
      self.firstFocus = True
  #
  def doCalibrate(self):
    # request useCalibration now
    self.robot.conWrite("encc\n")
  def reindex(self):
    # reindex calibration now - assumes calibration values are in place
    self.robot.conWrite("enci\n")
  def runMotors(self):
    # start motors with the requested motor voltage
    if (self.ui.encoder_run_motors.isChecked()):
      self.robot.conWrite("mote 1 1\n")
      # set motor voltage
      v = str(self.ui.encoder_motor_voltage.value())
      self.robot.conWrite("motv " + str(v) + " " + str(v) +"\n")
      self.motorVoltageSet = True
    else:
      self.robot.conWrite("mote 0 0\n")
      # set motor voltage
      self.robot.conWrite("motv 0 0\n")
      self.motorVoltageSet = False
  def collectTiming(self):
    if (self.ui.encoder_collect.isChecked()):
      self.robot.conWrite("enec=1\n")
    else:
      self.robot.conWrite("enec=0\n")
  def calibrationUse(self):
    if (self.ui.encoder_use_calibration.isChecked()):
      self.robot.conWrite("eneu=1\n")
    else:
      self.robot.conWrite("eneu=0\n")
  def autoCalibrate(self):
    if (self.ui.encoder_auto_calibrate.isChecked()):
      self.robot.conWrite("enea=1\n")
    else:
      self.robot.conWrite("enea=0\n")
    
  def justConnected(self):
    self.lastTab = "none"


    
