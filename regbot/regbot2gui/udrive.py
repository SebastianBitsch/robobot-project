#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Drive information voltage, current, encoder pose, velocity, etc.
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
#import math




class UDrive(object):
  motorCurrent = [0.0, 0.0]
  motorVolt = [0.0, 0.0]
  motorEncoder = [0, 0]
  wheelVelocity = [0.0, 0.0]
  wheelPos = [0.0, 0.0]
  pose = [0.0, 0.0, 0.0] # in m,m,radians
  tilt = 0.0 # in radians
  distance = 0.0
  battery = 0.0
  lock = threading.RLock()
  dataReadMca = False
  dataReadMcv = False
  dataReadEnc = False
  dataReadWve = False
  dataReadWpo = False
  dataReadPse = False
  dataReadBat = False
  lastDataRequestTime = time.time()
  nextDataRequest = 0
  lastTab = ""
  cpu_load = 1000
    #
  pwp = 0 # handle for plot window
  pwt = 0 # handle for plot window tilt
  cg = 0  # position plot
  cgt = 0 # tilt plot
  idxMax = 1000;
  data = np.zeros((2, idxMax))
  newPos = False
  datat = np.zeros(100)    # tilt data
  a1 = 0 # pose arrow
  idx = 0 # index (count of) pose items
  idxt = 0 # index to tilt data
  minx = -1.0
  miny = -1.0
  maxx = 2.0
  maxy = 2.0
  #
  def __init__(self, robot, ui):
    self.robot = robot
    self.ui = ui

  def readData(self, gg):
    used = True
    self.lock.acquire()
    if True: #try:
      if gg[0] == "mca":
        self.motorCurrent[0] = float(gg[1])
        self.motorCurrent[1] = float(gg[2])
        self.dataReadMca = True
      elif gg[0] == "mcv":
        self.motorVolt[0] = float(gg[1])
        self.motorVolt[1] = float(gg[2])
        self.dataReadMcv = True
      elif gg[0] == "enc":
        e = int(gg[1],16)
        if (e > 0x7fffffff):
          self.motorEncoder[0] = e - 0x100000000 
        else:
          self.motorEncoder[0] = e
        e = int(gg[2],16)
        if (e > 0x7fffffff):
          self.motorEncoder[1] = e - 0x100000000 
        else:
          self.motorEncoder[1] = e
        self.dataReadEnc = True
      elif gg[0] == "wve":
        self.wheelVelocity[0] = float(gg[1])
        self.wheelVelocity[1] = float(gg[2])
        self.dataReadWve = True
      elif gg[0] == "wpo":
        self.wheelPos[0] = float(gg[1])
        self.wheelPos[1] = float(gg[2])
        self.dataReadWpo = True
      elif gg[0] == "pse":
        self.pose[0] = float(gg[1])
        self.pose[1] = float(gg[2])
        self.pose[2] = float(gg[3])
        self.distance = float(gg[4])
        if (len(gg) > 4):
          # there is a tilt value
          self.tilt = float(gg[5])
          # add to plot array
          self.datat[self.idxt] = self.tilt * 180 / np.pi
          # increase tilt index
          if (self.idxt >= 100 - 1):
            self.idxt = 0
          else:
            self.idxt += 1
        # need for new pose history
        if abs(self.pose[0] - self.data[0,self.idx]) > 0.001 or abs(self.pose[1] - self.data[1,self.idx]) > 0.001:
          self.idx += 1
          if (self.idx >= self.idxMax):
            # remove old data to get space in array
            #print("drive overflow idx==" + str(self.idx) + ", max= " + str(self.idxMax))
            self.idx=self.idxMax // 2 # Integer division
            #print("drive at idx=" + str(self.idx) + ", data length= " + str(self.data.shape))
            self.data = np.append(self.data[:,self.idx:], np.zeros((2,self.idxMax - self.idx)), axis=1) 
            #self.data = self.data[:,self.idx:]
          self.data[0, self.idx] = self.pose[0]
          self.data[1, self.idx] = self.pose[1]
          self.newPos = True
        if (self.pose[0] > self.maxx):
          self.maxx = self.pose[0]
        elif (self.pose[0] < self.minx):
          self.minx = self.pose[0]  
        if (self.pose[1] > self.maxy):
          self.maxy = self.pose[1]
        elif (self.pose[1] < self.miny):
          self.miny = self.pose[1]  
        self.dataReadPse = True
      elif gg[0] == "bat":
        self.battery = float(gg[1])
        self.dataReadBat = True
      else:
        used = False
    #except:
      #print("UDrive: data read error - skipped a " + gg[0])
      #pass
    self.lock.release()
    return used
  
  
  def timerUpdate(self):
    if (self.dataReadBat):
      self.dataReadBat = False
      self.ui.val_batt.setValue(self.battery)
      self.ui.cpu_load.setText(str(self.cpu_load))
    if self.dataReadEnc:
      self.dataReadEnc = False
      self.ui.robot_enc_left.setValue(self.motorEncoder[0])
      self.ui.robot_enc_right.setValue(self.motorEncoder[1])
      self.ui.encoder_left_count.setText(str(self.motorEncoder[0]))
      self.ui.encoder_right_count.setText(str(self.motorEncoder[1]))      
    if self.dataReadMca:
      self.dataReadMca = False
      self.ui.robot_current_left.setValue(self.motorCurrent[0])
      self.ui.robot_current_right.setValue(self.motorCurrent[1])
    if self.dataReadMcv:
      self.dataReadMcv = False
      #self.ui.robot_volt_left.setValue(self.motorVolt[0])
      #self.ui.robot_volt_right.setValue(self.motorVolt[1])
    if self.dataReadWve:
      self.dataReadWve = False
      self.ui.robot_wheel_vel_left.setValue(self.wheelVelocity[0])
      self.ui.robot_wheel_vel_right.setValue(self.wheelVelocity[1])
    if self.dataReadWpo:
      self.dataReadWpo = False
      #self.ui.robot_wheel_pos_left.setValue(self.wheelPos[0])
      #self.ui.robot_wheel_pos_right.setValue(self.wheelPos[1])
    if self.dataReadPse:
      self.dataReadPse = False
      self.ui.robot_pose_x.setValue(self.pose[0])
      self.ui.robot_pose_y.setValue(self.pose[1])
      self.ui.robot_pose_h.setValue(self.pose[2])
      self.ui.robot_pose_h_2.setValue(self.pose[2]*180.0/np.pi);
      self.ui.robot_distance.setValue(self.distance)
      self.ui.robot_tilt.setValue(self.tilt)
      self.ui.robot_tilt_2.setValue(self.tilt*180.0/np.pi);
      self.ui.val_imu_tilt.setValue(self.tilt)
      self.ui.val_imu_tilt_2.setValue(self.tilt*180.0/np.pi)
    # show new robot pose
    if (self.newPos):
      # send data slize up to index to be displayed
      self.cg.setData(x=self.data[0,:self.idx + 1], y=self.data[1,:self.idx + 1])
      #print("drive " + str(self.idx) + " pos " + str(self.data[0,self.idx]) + ", " + str(self.data[1,self.idx]))
      self.newPos = False
      # self.cg.setLimits(self.minx, self.maxx, self.miny, self.maxy)
      self.pwg.removeItem(self.a1)
      self.a1 = pg.ArrowItem(angle=180 - self.pose[2]*180/np.pi)
      self.a1.setPos(self.pose[0],self.pose[1])
      self.pwg.addItem(self.a1)
    #self.a1.setData(self.pose[2])
    # show new tilt angle
    self.cgt.setData(self.datat)
    # request new data
    if (self.lastTab != self.robot.currentTab):
      if self.robot.info.talkToBridge:
        if self.robot.currentTab == "robot":
          # subscribe fast
          print("UDrive::timer -> robot - subscribe\n")
          self.robot.conWrite("pse subscribe 2\n")
          #self.robot.conWrite("u22\n")
          self.robot.conWrite("enc subscribe 3\n")
          self.robot.conWrite("u19\n")
          self.robot.conWrite("wve subscribe 2\n")
          self.robot.conWrite("u20\n")
          self.robot.conWrite("mca subscribe 2\n")
          self.robot.conWrite("u18\n")
        elif self.lastTab == "robot":
          # slow down traffic
          print("UDrive::timer -> robot - un-subscribe\n")
          self.robot.conWrite("pse subscribe 0\n") # pose
          self.robot.conWrite("enc subscribe 0\n") # encoder
          self.robot.conWrite("wve subscribe 0\n") # velocity
          self.robot.conWrite("mca subscribe 0\n") # motor current
      else: # direct to REGBOT
        if self.robot.currentTab == "robot":
          self.robot.conWrite("sub 1 1 1\n") # pose subscribe
          self.robot.conWrite("sub 1 2 6\n") # motor current subscribe
          self.robot.conWrite("sub 1 2 7\n") # velocity subscribe
        elif self.lastTab == "robot": # unsubscribe
          self.robot.conWrite("sub 0 1 1\n") # pose unsubscribe        
          self.robot.conWrite("sub 0 2 6\n") # motor current unsubscribe        
          self.robot.conWrite("sub 0 2 7\n") # velocity unsubscribe
      self.lastTab = self.robot.currentTab
    if self.robot.currentTab == "robot" and not self.robot.info.talkToBridge:
      #print("#tab_2 has focus")
      if (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected():
        if time.time() - self.lastDataRequestTime > 1.5:
          self.robot.conWrite("u4\n")
          self.lastDataRequestTime = time.time()
        else:
          if (self.nextDataRequest == 1):
            self.robot.conWrite("u19\n") # encoder
          elif self.nextDataRequest == 2:
            self.robot.conWrite("u18\n") # motorCurrent
          #elif self.nextDataRequest == 3:
            #self.robot.conWrite("u20\n") # wheel velocity
          #elif self.nextDataRequest == 4:
            #self.robot.conWrite("u22\n") # pose
          else:
            self.nextDataRequest = 0
          self.nextDataRequest += 1
    pass
  #
  def poseReset(self):
    self.idx = 0
    self.data = np.zeros((2, self.idxMax))
    self.newPos = True
    self.timerUpdate()
    print("drive pose reset")
  def initGraph(self):
    "initialize graph plot robot pose"
    # pose
    self.pwg = pg.PlotWidget(name='robot-pose',title='robot position')  ## giving the plots names allows us to link their axes together
    self.pwg.setLabel('bottom','x position','m')
    self.pwg.setLabel('left','y position','m')
    self.pwg.setWindowTitle('Pose')
    self.ui.robot_pose_layout.addWidget(self.pwg)
    self.cg = self.pwg.plot(pen='r',name='position m')
    #self.cg.setAspectLocked()
    vb = self.cg.getViewBox()
    #self.pwg.setLimits(minXRange=3.0, minYRange=3.0)
    #self.pwg.setLimits(-1.0, 2.0, -1.0, 2.0)
    self.pwg.setAspectLocked()
    self.cg.setData(x=self.data[0], y=self.data[1])
    # pose arrow
    self.a1 = pg.ArrowItem(angle=60)
    self.a1.setPos(0,0)
    self.pwg.addItem(self.a1)
    # tilt
    self.pwt = pg.PlotWidget(name='IMU-plot tilt',title='Tilt angle')  ## giving the plots names allows us to link their axes together
    self.pwt.setWindowTitle('IMU tilt')
    #self.pwt.setLabel('bottom','sample')
    self.pwt.setLabel('left','',' deg')
    self.ui.robot_graph_layout_tilt.addWidget(self.pwt)
    self.cgt = self.pwt.plot(pen='r',name='degres')
    self.cgt.setData(self.datat)
  #
  def justConnected(self):
    self.lastTab = "none"
