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


class UImu(object):
  "Class to handle IMU data"
  gyro = [0.0, 0.0, 0.0]
  gyroOffset = [0, 0, 0]
  gyroOffsetOK = False;
  acc = [0.0, 0.0, 0.0]
  imudataReadAcw = False
  imudataReadGyo = False
  imudataReadGyw = False
  lock = threading.RLock()
  # plot if IMU data - test
  pwg = 0 # handle for plot window
  pwa = 0 # handle for plot window acc
  #pwt = 0 # handle for plot window tilt
  cg = 0
  cga = 0
  #cgt = 0
  idx = 0
  data = np.zeros((3,100))
  dataa = np.zeros((3,100))
  lastDataRequestTime = time.time()
  lastDataRequest = 0
  lastTab = 'notme' # to turn on and off streaming of data
  waitingForData = False


  #datat = np.zeros(100)
  def __init__(self, robot, ui):
    self.robot = robot
    self.ui = ui
  #
  def initGraph(self):
    "initialize graph plot, IMU data"
    self.pwg = pg.PlotWidget(name='IMU-plot gyro',title='Gyro')  ## giving the plots names allows us to link their axes together
    #self.pwg.setWindowTitle('IMU gyro')
    self.pwg.setLabel('left','rotation velocity','deg/s')
    self.pwg.addLegend()    
    self.ui.robot_graph_layout_gyro.addWidget(self.pwg)
    self.cg = [self.pwg.plot(pen='r',name='x'), self.pwg.plot(pen='b',name='y'), self.pwg.plot(pen='g',name='z')]
    self.cg[0].setData(self.data[0])
    self.cg[1].setData(self.data[1])
    self.cg[2].setData(self.data[2])
    # acc
    self.pwa = pg.PlotWidget(name='IMU-plot acc',title='Accelerometer')  ## giving the plots names allows us to link their axes together
    #self.pwa.setWindowTitle('IMU accelerometer')
    self.pwa.setLabel('left','acceleration','m/s^2')
    self.pwa.addLegend()    
    self.ui.robot_graph_layout_acc.addWidget(self.pwa)
    self.cga = [self.pwa.plot(pen='r',name='x'), self.pwa.plot(pen='b',name='y'), self.pwa.plot(pen='g',name='z')]
    self.cga[0].setData(self.dataa[0])
    self.cga[1].setData(self.dataa[1])
    self.cga[2].setData(self.dataa[2])
    # tilt
    #self.pwt = pg.PlotWidget(name='IMU-plot tilt')  ## giving the plots names allows us to link their axes together
    #self.pwt.setWindowTitle('IMU tilt')
    ##self.pwt.addLegend()    
    #self.ui.robot_graph_layout_tilt.addWidget(self.pwt)
    #self.cgt = self.pwt.plot(pen='r',name='x')
    #self.cgt.setData(self.datat)
  #
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "gyw":
        self.gyro[0] = float(gg[1])
        self.gyro[1] = float(gg[2])
        self.gyro[2] = float(gg[3])
        self.data[0,self.idx] = self.gyro[0]
        self.data[1,self.idx] = self.gyro[1]
        self.data[2,self.idx] = self.gyro[2]
        if self.idx < 100 - 1:
          self.idx += 1
        else:
          self.idx = 0
        self.imudataReadGyw = True
        self.waitingForData = False
      elif gg[0] == "acw": 
        self.acc[0] = float(gg[1])
        self.acc[1] = float(gg[2])
        self.acc[2] = float(gg[3])
        self.dataa[0,self.idx] = self.acc[0]
        self.dataa[1,self.idx] = self.acc[1]
        self.dataa[2,self.idx] = self.acc[2]
        self.imudataReadAcw = True
        self.waitingForData = False
      elif gg[0] == "gyo":
        self.gyroOffset[0] = float(gg[1])
        self.gyroOffset[1] = float(gg[2])
        self.gyroOffset[2] = float(gg[3])
        self.gyroOffsetOK = int(gg[4], 10)
        self.imudataReadGyo = True
        self.waitingForData = False
      else:
        used = False
    except:
      print("UImu: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def timerUpdate(self):
    if self.imudataReadAcw:
      self.imudataReadAcw = False
      self.ui.val_acc.setValue(self.acc[0])
      self.ui.val_acc_2.setValue(self.acc[1])
      self.ui.val_acc_3.setValue(self.acc[2])
      self.cga[0].setData(self.dataa[0])
      self.cga[1].setData(self.dataa[1])
      self.cga[2].setData(self.dataa[2])
    if self.imudataReadGyw:
      self.imudataReadGyw = False
      self.ui.val_gyro.setValue(self.gyro[0])
      self.ui.val_gyro_2.setValue(self.gyro[1])
      self.ui.val_gyro_3.setValue(self.gyro[2])
      self.cg[0].setData(self.data[0])
      self.cg[1].setData(self.data[1])
      self.cg[2].setData(self.data[2])
    if self.imudataReadGyo:
      self.imudataReadGyo = False
      self.ui.val_gyro_offset_x.setValue(self.gyroOffset[0])
      self.ui.val_gyro_offset_y.setValue(self.gyroOffset[1])
      self.ui.val_gyro_offset_z.setValue(self.gyroOffset[2])
      self.ui.imu_gyro_offset_done.setChecked(self.gyroOffsetOK)
    # tilt set from drive.py
    #self.cgt.setData(self.datat)
    if self.robot.wifiConnected or self.robot.isConnected:
      if self.robot.currentTab == "imu" and self.lastTab != "me":
        # subscribe 
        if self.robot.info.talkToBridge:
          # start subscription from bridge
          self.robot.conWrite("gyw subscribe 2\n") # gyro
          self.robot.conWrite("gyo subscribe 1\n") # gyro offset
          self.robot.conWrite("acw subscribe 2\n") # accelerometer   pse
          self.robot.conWrite("pse subscribe 2\n") # pose
        else: # REGBOT direct
          self.robot.conWrite("sub 1 1 4\n") # acc subscribe
          self.robot.conWrite("sub 1 1 5\n") # acc
          self.robot.conWrite("sub 1 1 1\n") # pose
        self.lastTab='me'
      elif self.lastTab == 'me' and self.robot.currentTab != 'imu':
        # stop subscribe for imu data
        # "m=msg:0=hbt, 1=pose,2=IR,3=edge,4=acc,5=gyro,6=amp,7=vel
        if self.robot.info.talkToBridge:
          # stop subscription from bridge too
          self.robot.conWrite("gyw subscribe 0\n")
          self.robot.conWrite("gyo subscribe 0\n") # gyro offset
          self.robot.conWrite("acw subscribe 0\n")
          self.robot.conWrite("pse subscribe 0\n") # pose
        else: # REGBOT direct
          self.robot.conWrite("sub 0 1 1\n") # pose
          self.robot.conWrite("sub 0 1 4\n") # acc unsubscribe
          self.robot.conWrite("sub 0 1 5\n") # gyro unsubscribe
        self.lastTab = 'notme'
      dt = time.time() - self.lastDataRequestTime
      if self.lastTab == "me":
        if (dt > 0.02 and not self.waitingForData):
          self.lastDataRequestTime = time.time()
          self.waitingForData = True
          if (self.lastDataRequest == 0):
            self.robot.conWrite("u17\n") # gyro offset
          #elif (self.lastDataRequest == 1):
            #self.robot.conWrite("u15\n") # acc
          #elif (self.lastDataRequest == 2):
            #self.robot.conWrite("u16\n") # gyro 
          #elif (self.lastDataRequest == 3):
            #self.robot.conWrite("u22\n") # pose
          else:
            self.lastDataRequest = -1
            self.waitingForData = False
          self.lastDataRequest += 1
        elif dt > 3:
          # timeout
          self.waitingForData = False
        #print("IMU wait=" + str(self.waitingForData) + ", req=" + str(self.lastDataRequest) + ", dt=" + str(dt))

  def justConnected(self):
    self.lastTab = "none"
