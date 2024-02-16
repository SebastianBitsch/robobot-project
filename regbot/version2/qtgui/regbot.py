#!/usr/bin/python
# -*- coding: utf-8 -*-

# "$Rev: 363 $"
CLIENT_REV = "$Id: regbot.py 363 2016-08-04 09:55:13Z jcan $"

import sys
import os
#from PyQt4 import QtGui, QtCore, Qt
from pyqtgraph.Qt import QtGui, QtCore
#from PyQt5.QtWidgets import *
#from PyQt5.QtGui import *
# pyuic4 regbot.ui > regbotdesign.py
from regbotdesign import Ui_regbot
import serial
#import thread
import threading
import time
import string
import ConfigParser
import random
import math
import socket
from time import sleep
#from fcntl import fcntl, F_GETFL, F_SETFL
#from os import O_NONBLOCK, read

import pyqtgraph as pg
import numpy as np
from pyqtgraph.dockarea import *
import timeit

#from uirdistance import UIRDistance

# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////
# ///////////////////////////////////////////////////

class ULog(object):
  log_allow = True
  log_lms = 0 # mission
  log_lac = 0 # acc
  log_lgy = 0 # gyro
  log_lma = 0 # motor current
  log_lvr = 0 # motor velocity reference
  log_lmv = 0 # motor voltage
  log_lmr = 0 # rotation in radians
  log_lme = 0 # encoder
  log_lpo = 0 # pose
  log_line = 0 # line sensor
  log_dist = 0 # line sensor
  log_lbt = 0 # battery
  #log_lbo = 0 # barometer not used
  log_lbc = 0 # log of balance control details
  log_lct = 0 # control time in us
  log_lin = 0 # interval
  log_ltr = 0; # turn rate
  log_lcn = [0,0] # log buffer rows and max rows
  log_lex = 0
  log_set_manually = False
  logFlagRead = False
  logDataRead = False
  lock = threading.RLock()
  logList = ""
  logData = ""
  #
  def showData(self):
    if self.logFlagRead:
      mymw.ui.log_buf_cnt.setText("used " + str(self.log_lcn[0]) + "/" + str(self.log_lcn[1]))
      mymw.ui.log_buffer_time.setText("Log time " + str(self.log_lin * self.log_lcn[1] / 1000.0) + " sec")
      #if not self.log_set_manually:
      self.lock.acquire()
      mymw.ui.log_lms.setChecked(self.log_lms)
      mymw.ui.log_lac.setChecked(self.log_lac)
      mymw.ui.log_lgy.setChecked(self.log_lgy)
      mymw.ui.log_lma.setChecked(self.log_lma)
      mymw.ui.log_lvr.setChecked(self.log_lvr)
      mymw.ui.log_lmv.setChecked(self.log_lmv)
      mymw.ui.log_lmr.setChecked(self.log_lmr)
      mymw.ui.log_lme.setChecked(self.log_lme)
      mymw.ui.log_turn_rate.setChecked(self.log_ltr)
      mymw.ui.log_lpo.setChecked(self.log_lpo)
      mymw.ui.log_line.setChecked(self.log_line)
      mymw.ui.log_distance.setChecked(self.log_dist)
      mymw.ui.log_lbt.setChecked(self.log_lbt)
      mymw.ui.log_lbc.setChecked(self.log_lbc)
      mymw.ui.log_lct.setChecked(self.log_lct)
      mymw.ui.log_lex.setChecked(self.log_lex)
      mymw.ui.log_interval.setValue(self.log_lin)
      mymw.ui.log_allow.setChecked(self.log_allow)
      #mymw.ui.log_lbo.setChecked(self.log_lbo)
      self.lock.release()
    if (self.logDataRead):
      self.lock.acquire()
      mymw.ui.log_view.setText(self.logList + self.logData)
      self.logDataRead = False
      self.lock.release()

  #
  def readData(self, gg, line):
    dataUsed = True
    self.lock.acquire()
    try:
      if (gg[0][0] == 'l'):
        if gg[0] == "lms":
          self.log_lms = int(gg[1],0)
        elif gg[0] == "lac":
          self.log_lac = int(gg[1],0)
        elif gg[0] == "lgy":
          self.log_lgy = int(gg[1],0)
        elif gg[0] == "lma":
          self.log_lma = int(gg[1],0)
        elif gg[0] == "lvr":
          self.log_lvr = int(gg[1],0)
        elif gg[0] == "lmv":
          self.log_lmv = int(gg[1],0)
        elif gg[0] == "lmr":
          self.log_lmr = int(gg[1],0)
        elif gg[0] == "lme":
          self.log_lme = int(gg[1],0)
        elif gg[0] == "lpo":
          self.log_lpo = int(gg[1],0)
        elif gg[0] == "ltr":
          self.log_ltr = int(gg[1],0)
        elif gg[0] == "line":
          self.log_line = int(gg[1],0)
        elif gg[0] == "ldi":
          self.log_dist = int(gg[1],0)
        elif gg[0] == "lbt":
          self.log_lbt = int(gg[1],0)
        #elif gg[0] == "lbo":
          #self.log_lbo = int(gg[1],0)
        elif gg[0] == "lbc":
          self.log_lbc = int(gg[1],0)
        elif gg[0] == "lct":
          self.log_lct = int(gg[1],0)
        elif gg[0] == "lex":
          self.log_lex = int(gg[1],0)
        # log interval
        elif gg[0] == "lin":
          self.log_lin = int(gg[1],0)
          self.log_allow = int(gg[2],0)
        # log buffer
        elif gg[0] == "lcn":
          self.log_lcn[0] = int(gg[1],0)
          self.log_lcn[1] = int(gg[2],0)
          # this should be the last, so time to update
          self.logFlagRead = True
        else:
          dataUsed = False
      elif (gg[0][0] == '%'):
        self.logList += line
        self.logDataRead = True
      elif ((gg[0][0] >= '0' and gg[0][0] <= '9') or gg[0][0] == '.'):
        self.logData += line
        self.logDataRead = True
      else:
        dataUsed = False
    except:
      print("ULog: data read error - skipped a " + gg[0] + " from " + line)
      pass
    self.lock.release()
    return dataUsed
  def logSetManually(self):
    if (not self.log_set_manually):
      self.log_set_manually = True
      mymw.ui.log_flag_apply.setEnabled(True)
  def logSave(self):
    try:
      f = open(mymw.ui.log_filename.text(), "w")
      if (mymw.ui.log_save_header.isChecked()):
        f.write('%% logfile from robot ' + mymw.ui.robot_id_main.text() + '\n')
        f.write(self.logList)
      if (mymw.ui.log_save_config.isChecked()):
        f.write('%% Configuration as seen in client (assumed updated from robot)\n')
        fil = open('.regbotConfigTemp.ini', 'r');
        for line in fil:
          f.write('% ' + line)
        fil.close()
        f.write('%% data log\n')
      f.write(self.logData)
      f.close()
    except:
      mymw.ui.statusbar.showMessage("Failed to open file " + mymw.ui.log_filename.text() + "!", 3000)
  def logClear(self):
    self.logData = ""
    self.logList = ""
    self.logDataRead = True

#//////////////////////////////////////////////////////
#//////////////////////////////////////////////////////
#//////////////////////////////////////////////////////
#//////////////////////////////////////////////////////

class UImu(object):
  "Class to handle IMU data"
  gyro = [0.0, 0.0, 0.0]
  gyroOffset = [0, 0, 0]
  gyroOffsetOK = False;
  acc = [0.0, 0.0, 0.0]
  imudataRead = False
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
  #datat = np.zeros(100)
  #
  def initGraph(self):
    "initialize graph plot, IMU data"
    self.pwg = pg.PlotWidget(name='IMU-plot gyro',title='Gyro')  ## giving the plots names allows us to link their axes together
    #self.pwg.setWindowTitle('IMU gyro')
    self.pwg.setLabel('left','rotation velocity','deg/s')
    self.pwg.addLegend()    
    mymw.ui.robot_graph_layout_gyro.addWidget(self.pwg)
    self.cg = [self.pwg.plot(pen='r',name='x'), self.pwg.plot(pen='b',name='y'), self.pwg.plot(pen='g',name='z')]
    self.cg[0].setData(self.data[0])
    self.cg[1].setData(self.data[1])
    self.cg[2].setData(self.data[2])
    # acc
    self.pwa = pg.PlotWidget(name='IMU-plot acc',title='Accelerometer')  ## giving the plots names allows us to link their axes together
    #self.pwa.setWindowTitle('IMU accelerometer')
    self.pwa.setLabel('left','acceleration','m/s^2')
    self.pwa.addLegend()    
    mymw.ui.robot_graph_layout_acc.addWidget(self.pwa)
    self.cga = [self.pwa.plot(pen='r',name='x'), self.pwa.plot(pen='b',name='y'), self.pwa.plot(pen='g',name='z')]
    self.cga[0].setData(self.dataa[0])
    self.cga[1].setData(self.dataa[1])
    self.cga[2].setData(self.dataa[2])
    # tilt
    #self.pwt = pg.PlotWidget(name='IMU-plot tilt')  ## giving the plots names allows us to link their axes together
    #self.pwt.setWindowTitle('IMU tilt')
    ##self.pwt.addLegend()    
    #mymw.ui.robot_graph_layout_tilt.addWidget(self.pwt)
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
      elif gg[0] == "acw": 
        self.acc[0] = float(gg[1])
        self.acc[1] = float(gg[2])
        self.acc[2] = float(gg[3])
        self.dataa[0,self.idx] = self.acc[0]
        self.dataa[1,self.idx] = self.acc[1]
        self.dataa[2,self.idx] = self.acc[2]
      elif gg[0] == "gyo":
        self.gyroOffset[0] = float(gg[1])
        self.gyroOffset[1] = float(gg[2])
        self.gyroOffset[2] = float(gg[3])
        self.gyroOffsetOK = int(gg[4], 10)
        self.imudataRead = True
      else:
        used = False
    except:
      print("UImu: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    if self.imudataRead:
      self.imudataRead = False
      self.lock.acquire()
      mymw.ui.val_acc.setValue(self.acc[0])
      mymw.ui.val_acc_2.setValue(self.acc[1])
      mymw.ui.val_acc_3.setValue(self.acc[2])
      mymw.ui.val_gyro.setValue(self.gyro[0])
      mymw.ui.val_gyro_2.setValue(self.gyro[1])
      mymw.ui.val_gyro_3.setValue(self.gyro[2])
      mymw.ui.val_gyro_offset_x.setValue(self.gyroOffset[0])
      mymw.ui.val_gyro_offset_y.setValue(self.gyroOffset[1])
      mymw.ui.val_gyro_offset_z.setValue(self.gyroOffset[2])
      mymw.ui.imu_gyro_offset_done.setChecked(self.gyroOffsetOK)
      self.cg[0].setData(self.data[0])
      self.cg[1].setData(self.data[1])
      self.cg[2].setData(self.data[2])
      self.cga[0].setData(self.dataa[0])
      self.cga[1].setData(self.dataa[1])
      self.cga[2].setData(self.dataa[2])
      #self.cgt.setData(self.datat)
      self.lock.release()
  
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

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
  dataRead = False
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
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "mca":
        self.motorCurrent[0] = float(gg[1])
        self.motorCurrent[1] = float(gg[2])
      elif gg[0] == "mcv":
        self.motorVolt[0] = float(gg[1])
        self.motorVolt[1] = float(gg[2])
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
      elif gg[0] == "wve":
        self.wheelVelocity[0] = float(gg[1])
        self.wheelVelocity[1] = float(gg[2])
      elif gg[0] == "wpo":
        self.wheelPos[0] = float(gg[1])
        self.wheelPos[1] = float(gg[2])
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
        # need for ned pose history
        if abs(self.pose[0] - self.data[0,self.idx]) > 0.001 or abs(self.pose[1] - self.data[1,self.idx]) > 0.001:
          self.idx += 1
          if (self.idx >= self.idxMax):
            # remove old data to get space in array
            self.idx=self.idxMax/2
            # reuse newest part and fill with zeros
            self.data = np.append(self.data[:,self.idx:], np.zeros((2,self.idxMax - self.idx)), axis=1) 
            #print("drive " + str(self.idx) + " reduced now " + str(self.data))
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
      elif gg[0] == "bat":
        self.battery = float(gg[1])
        self.dataRead = True
      else:
        used = False
    except:
      print("UDrive: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    if (self.dataRead):
      self.dataRead = False
      mymw.ui.val_batt.setValue(self.battery)
      mymw.ui.robot_enc_left.setValue(self.motorEncoder[0])
      mymw.ui.robot_enc_right.setValue(self.motorEncoder[1])
      mymw.ui.robot_current_left.setValue(self.motorCurrent[0])
      mymw.ui.robot_current_right.setValue(self.motorCurrent[1])
      mymw.ui.robot_volt_left.setValue(self.motorVolt[0])
      mymw.ui.robot_volt_right.setValue(self.motorVolt[1])
      mymw.ui.robot_wheel_vel_left.setValue(self.wheelVelocity[0])
      mymw.ui.robot_wheel_vel_right.setValue(self.wheelVelocity[1])
      #mymw.ui.robot_wheel_pos_left.setValue(self.wheelPos[0])
      #mymw.ui.robot_wheel_pos_right.setValue(self.wheelPos[1])
      mymw.ui.robot_pose_x.setValue(self.pose[0])
      mymw.ui.robot_pose_y.setValue(self.pose[1])
      mymw.ui.robot_pose_h.setValue(self.pose[2])
      mymw.ui.robot_pose_h_2.setValue(self.pose[2]*180.0/math.pi);
      mymw.ui.robot_distance.setValue(self.distance)
      mymw.ui.robot_tilt.setValue(self.tilt)
      mymw.ui.robot_tilt_2.setValue(self.tilt*180.0/math.pi);
      mymw.ui.val_imu_tilt.setValue(self.tilt)
      mymw.ui.val_imu_tilt_2.setValue(self.tilt*180.0/math.pi)
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
  #
  def poseReset(self):
    self.idx = 0
    self.newPos = True
    #self.pwg.clear()
    self.showData()
    #self.data = np.array([[0.0],[0.0]])
    #if (self.cg != 0):
      #self.cg.setData(self.data[0,:self.idx + 1])
    print("drive pose reset")
  def initGraph(self):
    "initialize graph plot robot pose"
    # pose
    self.pwg = pg.PlotWidget(name='robot-pose',title='robot position')  ## giving the plots names allows us to link their axes together
    self.pwg.setLabel('bottom','x position','m')
    self.pwg.setLabel('left','y position','m')
    self.pwg.setWindowTitle('Pose')
    mymw.ui.robot_pose_layout.addWidget(self.pwg)
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
    mymw.ui.robot_graph_layout_tilt.addWidget(self.pwt)
    self.cgt = self.pwt.plot(pen='r',name='degres')
    self.cgt.setData(self.datat)
  #

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class UMissionLine(object):
  vel = 0
  velUse = False
  acc = 1
  accUse = False
  log = 0
  logUse = False
  bal = 0
  balUse = False
  tr = 0
  trUse = False
  llabel = 0
  lineRef = 0;
  lineLUse = False
  lineRUse = False
  lineWhiteUse = False
  lineWhite = False
  irSensorUse = False
  irSensor = 1
  irDistUse = False
  irDist = 0.2
  pos = 0
  posUse = False
  head = 0
  headUse = False
  gotoUse = False
  gotoDest = 0L
  # continue conditions
  dist = 0
  distUse = '\0'
  turn = 0
  turnUse = '\0'
  time = 0
  timeUse = False
  countUse = False
  count = 0L
  xingWhiteUse = '\0'
  xingBlackUse = '\0'
  xingBlackVal = 0;
  xingWhiteVal = 0;
  lineValidUse = False;
  lineValidVal = 0;  
  tiltUse = '\0'
  tiltValue = 0
  irDist1Use = '\0'
  irDist2Use = '\0'
  irDist1 = 0
  irDist2 = 0
  threadUse = False
  thread = 0
  eventSet = [False] * 5
  eventSetCnt = 0
  eventMask = [False] * 5
  eventMaskCnt = 0
  logFullUse = False
  # 
  valid = False
  showed = True
  def clear(self):
    self.velUse = False
    self.accUse = False
    self.logUse = False
    self.balUse = False
    self.trUse = False
    self.lineLUse = False
    self.lineRUse = False
    self.lineWhiteUse = False
    self.irSensorUse = False
    self.irDistUse = False
    self.gotoUse = False
    self.eventSet = [False] * 5
    self.eventSetCnt = False
    self.posUse = False
    self.headUse = False
    # continue conditions
    self.distUse = '\0'
    self.turnUse = '\0'
    self.timeUse = False
    self.countUse = False
    self.xingWhiteUse = '\0'
    self.xingBlackUse = '\0'
    self.lineValidUse = False
    self.tiltUse = '\0'
    self.irDist1Use = '\0'
    self.irDist2Use = '\0'
    self.threadUse = False
    self.eventMaskCnt = 0
    self.eventMask = [False] * 5
    self.logFullUse = False
    
  def toString(self):
    #print("# converting mission line to string")
    ms = ""
    mc = ""
    if (self.velUse):
      ms = ", vel=" + str(self.vel)
    if (self.accUse):
      ms = ms + ", acc=" + str(self.acc)
    if (self.trUse):
      ms = ms + ", tr=" + str(self.tr)
    if (self.lineLUse):
      ms = ms + ", edgel=" + str(self.lineRef)
    if (self.lineRUse):
      ms = ms + ", edger=" + str(self.lineRef)
    if (self.lineWhiteUse):
      ms = ms + ", white={:d}".format(int(self.lineWhite))
    if (self.logUse):
      ms = ms + ", log=" + str(self.log)
    if (self.balUse):
      ms = ms + ", bal={:d}".format(int(self.bal))
    if (self.irSensorUse):
      ms = ms + ", irsensor={:d}".format(int(self.irSensor))
    if (self.irDistUse):
      ms = ms + ", irdist=" + str(self.irDist)
    if (self.gotoUse):
      ms = ms + ", goto={:d}".format(int(self.gotoDest))
    if (self.llabel > 0):
      ms = ms + ", label={:d}".format(int(self.llabel))
    if (self.threadUse):
      ms = ms + ", thread={:d}".format(self.thread)
    if (self.posUse):
      ms = ms + ", topos={}".format(self.pos)
    if (self.headUse):
      ms = ms + ", head={:d}".format(self.head)
    if self.eventSetCnt > 0:
      for i in range(0, self.eventSetCnt):
        ms = ms + ", event={:d}".format(self.eventSet[i])
    if (len(ms) > 2):
      # remove first 2 characters
      ms = ms[2:]
    #print("# now ms is " + ms)
    if (self.distUse != '\0'):
      mc = mc + ", dist=" + str(self.dist)
    if (self.timeUse):
      mc = mc + ", time=" + str(self.time)
    if (self.turnUse != '\0'):
      mc = mc + ", turn=" + str(self.turn)
    if (self.countUse):
      mc = mc + ", count={:d}".format(int(self.count))
    if (self.xingBlackUse != '\0'):
      mc = mc + ", xb{}{:d}".format(self.xingBlackUse, int(self.xingBlackVal))
    if (self.xingWhiteUse != '\0'):
      mc = mc + ", xw{}{:d}".format(self.xingWhiteUse, int(self.xingWhiteVal))
    if (self.tiltUse != '\0'):
      mc = mc + ", tilt{}{}".format(self.tiltUse, self.tiltValue)
    if (self.lineValidUse):
      mc = mc + ", lv={:d}".format(int(self.lineValidVal))
    if (self.irDist1Use != '\0'):
      mc = mc + ", ir1{}{}".format(self.irDist1Use, self.irDist1)
    if (self.irDist2Use != '\0'):
      mc = mc + ", ir2{}{}".format(self.irDist2Use, self.irDist2)
    if self.eventMaskCnt > 0:
      for i in range(0,self.eventMaskCnt):
        mc = mc + ", event={:d}".format(self.eventMask[i])
    if self.logFullUse:
      mc = mc + ", log=0"
    if (len(mc) > 2):
      # collect parameter with condition
      ms = ms + ": " + mc[2:]
    return ms
  #
  def isFloat(self, val):
    isOK = True
    v2 = val
    if (v2[0] == '-' or v2[0] == '+'):
      v2 = val[1:]
    if not v2.replace('.','',1).isdigit():
      isOK = False
    return isOK
  #
  # decode mission line from robot
  # format like "vel=0,acc=3,event=1,event=2:time=0.2,ir1<0.5,ir2>0.6"
  def setFromLine(self, line):
    #print("# setting mission line from string: " + line)
    part = line.split(':')
    prepart = part[0].split(',')
    errstr = ""
    error = False
    val = []
    v0 = ""
    self.clear()
    if (line[0:2] == "<m"):
      part[0] = part[0][2:]
    prepart[0] = prepart[0].strip()
    if (len(prepart[0]) > 0):
      pass # there is an assignment part
      for j in range(0, len(prepart)):
        pass # print("  pre" + str(j) + " " + prepart[j])
        val = prepart[j].split('=')
        if len(val) != 2:
          error = True
          break
        v0 = val[0].strip().lower()
        v1 = val[1].strip()
        error = not self.isFloat(v1)
        if (error):
          break
        if v0 == 'vel':
          self.vel = float(v1)
          self.velUse = True
        elif v0 == 'acc':
          self.acc = float(v1)
          self.accUse = True
        elif v0 == 'tr':
          self.tr = float(v1)
          self.trUse = True
        elif v0 == 'edgel':
          self.lineRef = float(v1)
          self.lineLUse = True
          if self.lineRef > 2.0:
            self.lineRef = 2.0
          elif self.lineRef < -2.0:
            self.lineRef = -2.0
        elif v0 == 'edger':
          self.lineRef = float(v1)
          self.lineRUse = True
          if self.lineRef > 2.0:
            self.lineRef = 2.0
          elif self.lineRef < -2.0:
            self.lineRef = -2.0
        elif v0 == 'white':
          self.lineWhite = float(v1)
          self.lineWhiteUse = True
        elif v0 == 'log':
          self.log = float(v1)
          self.logUse = True
        elif v0 == 'bal':
          self.bal = float(v1)
          self.balUse = True
        elif v0 == 'irsensor':
          self.irSensor = int(v1)
          self.irSensorUse = True
        elif v0 == 'irdist':
          self.irDist = float(v1)
          self.irDistUse = True
        elif v0 == 'label':
          self.llabel = float(v1)
        elif v0 == 'goto':
          self.gotoDest = float(v1)
          self.gotoUse = True
        elif v0 == 'thread':
          self.thread = int(v1)
          self.threadUse = True
        elif v0 == 'topos':
          self.pos = float(v1)
          self.posUse = True
        elif v0 == 'head':
          self.head = float(v1)
          self.headUse = True
        elif v0 == 'event':
          iv1 = int(v1)
          if (iv1 >= 0) and (iv1 < 32):
            self.eventSet[self.eventSetCnt] = iv1
            self.eventSetCnt = self.eventSetCnt + 1
            # print("# set event mask " + str(self.eventSetCnt) + " value " + str(iv1) + " to true")
          else:
            error = True
            break
        else:
          error = True
          break
    # do condition part if there is one
    if (not error):
      if len(part) > 1 and not error:
        prepart = part[1].split(',')
        for j in range(0, len(prepart)):
          if (len(prepart[j]) > 2):
            valc = '='
            val = prepart[j].split(valc)
            if len(val) != 2:
              valc = '<'
              val = prepart[j].split(valc)
            if len(val) != 2:
              valc = '>'
              val = prepart[j].split(valc)
            if len(val) != 2:
              error = True
              break
            v0 = val[0].strip().lower()
            v1 = val[1].strip()
            error = not self.isFloat(v1)
            if (error):
              break
            if v0 == "dist":
              self.dist = float(v1)
              self.distUse = valc
            elif v0 == "turn":
              self.turn = float(v1)
              self.turnUse = valc
            elif v0 == "time":
              self.timeUse = valc
              self.time = float(v1)
            elif v0 == "count":
              self.countUse = valc
              self.count = float(v1)
            elif v0 == "xb":
              self.xingBlackUse = valc
              self.xingBlackVal = float(v1)
            elif v0 == "xw":
              self.xingWhiteUse = valc
              self.xingWhiteVal = float(v1)
            elif v0 == "lv":
              self.lineValidUse = '='
              self.lineValidVal = float(v1)
            elif v0 == "tilt":
              self.tiltUse = valc
              self.tiltValue = float(v1)
            elif v0 == "ir1":
              self.irDist1Use = valc
              self.irDist1 = float(v1)
              #print("#ir1 " + valc + self.irDist1Use + " value " + str(self.irDist1) + "\n")
            elif v0 == "ir2":
              self.irDist2Use = valc
              self.irDist2 = float(v1)
              #print("#ir2 " + valc + self.irDist2Use+ " value " + str(self.irDist2) + "\n")
            elif v0 == 'event':
              iv1 = int(v1)
              if (iv1 >= 0) and (iv1 < 32):
                self.eventMask[self.eventMaskCnt] = iv1
                self.eventMaskCnt = self.eventMaskCnt + 1
                #print("# set event mask " + str(self.eventMaskCnt) + " value " + str(iv1) + " to true")
              else:
                error = True
                break
            elif v0 == "log":
              self.logFullUse = True;
            else:
              error = True
              break;
      #else:
        ## there should always be a continue condition
        #error = True;
    if error:
      errstr = " near '" + prepart[j] + "'!"
      #if (len(val) >= 2):
        #errstr = errstr + " val[0]='" + v0 + "' val[1]='" + val[1] + "' + len(val)=" + str(len(val))
      self.valid = False
    else:
      self.valid = self.accUse or self.balUse or self.distUse != '\0' \
        or self.logUse or self.timeUse or self.trUse or self.turnUse != '\0' or self.velUse \
        or self.lineLUse or self.lineRUse or self.posUse or self.headUse \
        or self.lineWhiteUse or self.xingBlackUse != '\0' \
        or self.xingWhiteUse != '\0' or self.lineValidUse or self.tiltUse != '\0' \
        or self.irDistUse or self.irSensorUse or self.irDist1Use != '\0' or self.irDist2Use != '\0' \
        or self.gotoUse or self.eventSetCnt > 0 or self.eventMaskCnt > 0 or self.logFullUse
    return errstr
  #
  # end of UMissionLine


class UMission(object):
  startSwitch = False
  # current mission to run
  mission = 2
  missionState = 0
  missionLineState = 0
  missionThreadState = 0
  missionDataRead = True
  missionName = "mission name"
  missionManually = False
  lock = threading.RLock()
  mission_directory = ""
  mission_filename = "regbit_mission.mis"
  statusWhileRunning = 0;
  missionTextEdit = ""
  missionTextEditChaged = True
  # user mission lines
  mLines = []
  mLinesNewData = False
  about_box = None
  checkNr = 0
  def readData(self, gg, line):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "swv":
        self.startSwitch = int(gg[1],0)
      # control related items
      elif gg[0] == "mis":
        self.mission = int(gg[1],10)
        self.missionState = int(gg[2],10)
        self.missionLineState = int(gg[3],10)
        self.missionName = gg[4]
        self.statusWhileRunning = int(gg[5], 10)
        if (len(gg) > 6):
          self.missionThreadState = int(gg[6], 10)
        self.missionDataRead = True
      else:
        used = False
    except:
      print("UMission: data read error - skipped a " + gg[0] + ", len=" + str(len(gg)) + " expected 6")
      pass
    self.lock.release()
    return used
  def showData(self):
    if (self.missionTextEditChaged):
      self.missionTextEditChaged = False
      mymw.ui.mission_edit.setPlainText(self.missionTextEdit)
    if (self.missionDataRead or self.mLinesNewData or self.missionTextEditChaged):
      self.missionDataRead = False
      self.missionTextEditChaged = False
      self.lock.acquire()
      if (not self.missionManually):
        mymw.ui.main_mission_2.setValue(self.mission)
      mymw.ui.mission_name.setText(self.missionName)
      if (self.missionThreadState > 0 or self.missionLineState > 0):
        mymw.ui.main_mission_state.setValue(self.missionThreadState + self.missionLineState/100.0)
      else:
        mymw.ui.main_mission_state.setValue(self.missionState + self.missionLineState/100.0)
      #if (self.mLinesNewData):
        #self.mLinesNewData = False
        #j = 1
        #for i in self.mLines:
          #if (not i.showed):
            #i.showed = True
            #if i.threadUse:
              #mymw.ui.mission_edit.append(i.toString())
            #else:
              #mymw.ui.mission_edit.append("    " + i.toString())
            #j = j + 1
      mymw.ui.main_status_while_running.setChecked(self.statusWhileRunning)
      self.lock.release()
    pass
  def dataChangedManually(self):
    self.missionManually = True
  def saveMissionToFile(self):
    try:
      f = open(mymw.ui.mission_filename.text(), "w");
      f.write(mymw.ui.mission_edit.toPlainText())
      f.close()
      mymw.ui.statusbar.showMessage("Save mission file " + mymw.ui.mission_filename.text() + ".", 3000)
    except:
      mymw.ui.statusbar.showMessage("Failed to save mission file " + mymw.ui.mission_filename.text() + "!", 3000)
  def loadMissionFromFile(self):
    try:
      f = open(mymw.ui.mission_filename.text(), "r");
      #mymw.ui.mission_edit.clear()
      self.missionTextEdit += f.read()
      self.missionTextEditChaged = True
      #mymw.ui.mission_edit.setPlainText(f.read())
      f.close()
      mymw.ui.statusbar.showMessage("Loaded mission file " + mymw.ui.mission_filename.text() + ".", 3000)
    except:
      mymw.ui.statusbar.showMessage("Failed to open mission file " + mymw.ui.mission_filename.text() + "!", 3000)
    pass
  def setMissionFileName(self):
    mis = QtGui.QFileDialog.getOpenFileName(mymw,'directory and filename for mission file', mymw.ui.mission_filename.text(), 'mission (*.mis)')
    if (mis is not None and len(mis) > 0):
      print("mission '" + mis + "'")
      mymw.ui.mission_filename.setText(mis)
  def checkMission(self):
    error = False
    self.mLines = []
    m = mymw.ui.mission_edit.toPlainText() 
    lines = m.split('\n')
    for i in range(0,lines.count()):
      lin = str(lines[i])
      lin = lin.strip()
      if (len(lin) > 4 and lin[0] != '#' and lin[0] != ';'):
        ml = UMissionLine()
        es = ml.setFromLine(lin)
        if (len(es) > 0):
          error = True
          break
        else:
          self.mLines.append(ml)
    if error:
      # no valid mission
      mymw.ui.mission_edit_error.setPlainText(str(self.checkNr) + ": Error line " + str(i+1) + " " + es)
      self.mLines = []
    else:
      mymw.ui.mission_edit_error.setPlainText(str(self.checkNr) + ": No Error found (in " + str(lines.count()) + " lines)")
    self.checkNr = self.checkNr + 1
  #
  ## send compiled user mission to robot - called by top "save" button on mission pane
  def sendToRobot(self):
    if robot.con.isOpen():
      self.checkMission()
      if (len(self.mLines) > 0):
        print("mission is OK!")
        # clear old mission
        robot.conWrite("<clear\n")
        for ml in range(0, len(self.mLines)):
          robot.conWrite("<add " + self.mLines[ml].toString() + "\n")
      #self.conWrite("u4\r\n") # static robot info
      #self.conWrite("S=" + str(mymw.ui.main_push_interval.value()) + "\r\n")
      mymw.ui.main_mission_2.setValue(3)
    else:
      print("connection is not open")
  #
  def readUserMissionLine(self, rawLine):
    used = False
    line = rawLine.strip()
    if (line[0:2] == "<m"):
      lin = line[2:].strip()
      if (len(lin) > 4):
        ml = UMissionLine()
        es = ml.setFromLine(lin)
        if (len(es) > 0):
          print("Error line " + str(len(self.mLines)) + " " + es)
          # mymw.ui.mission_edit_error.setPlainText("Error line " + str(len(self.mLines)) + " " + es)
        else:
          self.mLines.append(ml)
          if ml.threadUse:
            self.missionTextEdit += ml.toString() + '\n'
          else:
            self.missionTextEdit += "    " + ml.toString() + '\n'
          self.missionTextEditChaged = True
          #ml.showed = False
          #self.mLinesNewData = True
          #print("This line should now be appended to mission_window: " + line)
      used = True
    return used
  #
  def getFromRobot(self):
    self.missionTextEdit += "# Got from robot:\n"
    self.missionTextEditChaged = True
    #mymw.ui.mission_edit.append("# got from robot:")
    robot.conWrite("<get\n")
  def helpbox(self):
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      #                 TOPOS sets a target position for this line, and will stop at that position (no implicit continue).<br />
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Mission setup</span></p>
                <p>Mission specification consist of mission lines,
                   each line consist of two (lower case) parts divided by ':'</p>
                <p><b>drive values : continue conditions</b>  (conditions are OR'ed)</p>
                <p>e.g.: vel=-0.2, acc=3.0 : dist=1, time=12<br />
                Drive backwards at a speed of 0.2m/s, accelerate with 3m/s2 for 1 meter (or max 12 seconds)</p>
                
                <p>
                <b>Drive values</b><br />
                VEL is velocity in m/s (or Volts if no controller) - positive is forward, 0=stop. uses last value if omitted.<br />
                ACC is acceleration limit in m/s2. Uses last value if omitted.<br />
                TR is turnradius in metre - positive, 0 is turn on the spot. In balance 0.07 is minimum<br />
                EDGER is following Right edge of line at -2..2 (in cm), positive is right <br />
                EDGEL is following Left  edge of line at -2..2 (in cm), positive is right <br />
                WHITE set to 1 if follow-line tape is white, else black.<br />
                LOG is log interval in milliseconds. Once started it continues until buffer is full or paused by LOG=0. 
                    Log can be restarted (with another interval).<br />
                BAL is balancing, uses last value if omitted.<br />
                IRSENSOR is IR-sensor to use for control (1 controls turn, 2 controls velocity).<br />
                IRDist is IR-distance to hold (selsor 1 to wall, sensor 2 to leader).<br />
                HEAD sets the reference heading in odometry coordinates.<br />
                LABEL is a label number that can be used by GOTO.<br />
                GOTO is a jump to the label number given. This can be limited by COUNT or any other condition<br />
                <b>Continue conditions</b><br />
                DIST is driven distance in this mission line - positive meters<br />
                TURN is the (max) angle turned in this mission line - degrees, positive is CCV<br />
                TIME is max time in this mission line - positive seconds<br />
                COUNT is used with GOTO and GOTO will be skipped when count is reached. Count is reset when line is skipped.<br />
                XB, XW is test for crossing black/white line, value is 0..20, 0 is true on no crossing, 1..20 is confidence in crossing (20 is highest). Works when in balance or front down only.<br />
                LV is test for valid line 0=true for no valid line, 1=true for valid line.<br/>
                IR1 is test for distance from side IR sensor.<br/>
                IR2 is test for distance from forward IR sensor.<br/>
                TILT is test for tilt angle (0 is balance point)<br/>
                If no condition, then continues right away<br/>
                </p>
                <hr />
                <p>
                <b>e.g.</b>: drive-turn-drive-turn at 2cm/s:</p>
                <p>
                vel=0.2, acc=1.5  : dist = 0.5, time=5<br />
                tr=0.15 : turn=90<br />
                : dist = 0.5<br />
                tr=0.15 : turn=90<br />
                vel=0 : time=1<br />
                e.g. wait 5 seconds<br />
                vel=0 : time=5
                <hr />
                <p><b>Save</b></p>
                <p>
                Transfer the mission <i>to the robot RAM</i> by pressing "Save" (top right),
                the mission will be lost by a reboot or power off, unless saved on robot!<br />
                Save the mission on the robot <i>flash memory (EE-prom)</i> by pressing "save on Robot" (left panel). <br/>
                NB! The space is limited (maybe 50 lines)!<br />
                Save the mission in a <i>text-file</i> use the bottom right save button.
                </p>''');
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()

  
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class URobotInfo(object):
  wheelbase = 0.0
  gear = 0.0
  pulsePerRev = 0
  wheelLRadius = 0.0
  wheelRRadius = 0.0
  version = ""
  robotID = 0
  balanceOffset = 0.0
  batteryUse = True
  batteryIdleVolt = 9.9
  name = "empty"
  wifiIP = "0.0.0.0"
  wifiMAC= "00:00:00:00:00:00"
  #wifiport = 24001
  


class UInfo(object):
  dataRead = True
  dataWifi = True
  dataClient = True
  robotID = 0
  lock = threading.RLock()
  robots = [URobotInfo]
  robot = robots[0]
  inEdit = False
  wifiSSID = "aaa"
  wifiGotIP = False
  wifiPortOpen = False
  wifiInEdit = False
  wifiPort = 24001
  client1 = 0
  client2 = 0
  client3 = 0
  client4 = 0
  client5 = 0
  wifiGood = 0
  wifiLost = 0
  wifiLoss = 0.0
  #
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
        self.robot = self.getRobot(self.robotID)
        self.robot.wheelbase = float(gg[2])
        self.robot.gear = float(gg[3])
        self.robot.pulsePerRev = int(gg[4], 10)
        self.robot.wheelLRadius = float(gg[5])
        self.robot.wheelRRadius = float(gg[6])
        self.robot.balanceOffset = float(gg[7])
        self.robot.batteryUse = int(gg[8], 0)
        self.robot.batteryIdleVolt = float(gg[9])
        self.robot.name = gg[10]
        #print("got robot " + gg[1] + " " + gg[10] + " " + self.robot.name)
        #if (self.robot.robotID != 20):
          #print("moved robot... to " + str(self.robot.robotID))
        self.dataRead = True
      elif gg[0] == "version":
        self.robot.version = gg[1]
        self.dataRead = True
      elif gg[0] == "wfi":
        self.wifiPortOpen = gg[1] == "99"
        self.wifiGotIP = gg[2] >= "3"
        # gg[3] ignored (reply status)
        self.robot.wifiIP = gg[4] + '.' + gg[5] + '.' + gg[6] + '.' + gg[7]
        # got also MAC
        self.robot.wifiMAC = gg[8]
        self.wifiport = int(gg[9], 0)
        self.wifiSSID = "no data"
        if (len(gg) > 9):
          self.wifiSSID = ""
          for i in range(10,len(gg)):
            self.wifiSSID += gg[i] + " "
          self.dataWifi = True
        #print("# got wfi")
      elif gg[0] == "wfc":
        self.client1 = int(gg[2], 10)
        self.client2 = int(gg[4], 10)
        self.client3 = int(gg[6], 10)
        self.client4 = int(gg[8], 10)
        self.client5 = int(gg[10], 10)
        self.wifiGood = int(gg[11], 10)
        self.wifiLost = int(gg[12], 10)
        self.dataClient = True
      else:
        used = False
    except:
      print("URobot: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  #
  def showData(self):
    if (self.dataRead):
      self.dataRead = False
      self.lock.acquire()
      if (not self.inEdit):
        # set new values
        #mymw.ui.robot_sw_version.setText("Robot sw " + self.robot.version)
        mymw.ui.robot_gear.setProperty("value", self.robot.gear)
        mymw.ui.robot_pulse_per_rev.setValue(self.robot.pulsePerRev)
        mymw.ui.robot_wheel_radius_left.setValue(self.robot.wheelLRadius)
        mymw.ui.robot_wheel_radius_right.setValue(self.robot.wheelRRadius)
        mymw.ui.robot_balance_offset.setValue(self.robot.balanceOffset)
        mymw.ui.save_id_on_robot.setEnabled(False)
        mymw.ui.robot_base.setValue(self.robot.wheelbase)
        mymw.ui.robot_id.setValue(self.robotID)
        mymw.ui.robot_id_main.setText(self.robot.name + " (" + str(self.robotID) + ")")
        mymw.ui.robot_on_battery.setChecked(self.robot.batteryUse)
        mymw.ui.robot_battery_idle_volt.setValue(self.robot.batteryIdleVolt)
        # and buttons
        mymw.ui.save_id_on_robot.setEnabled(False)
        mymw.ui.robot_cancel.setEnabled(False)
        mymw.ui.robot_edit.setEnabled(True)
      else:
        mymw.ui.save_id_on_robot.setEnabled(True)
        mymw.ui.robot_cancel.setEnabled(True)
        mymw.ui.robot_edit.setEnabled(False)
      self.lock.release()
    if (self.dataWifi):
      self.lock.acquire()
      #print("# showing wfi")
      if (not self.wifiInEdit):
        mymw.ui.wifi_got_ip.setChecked(self.wifiGotIP)
        mymw.ui.wifi_port_open.setChecked(self.wifiPortOpen)
        mymw.ui.wifi_port.setText(str(self.wifiPort))
        mymw.ui.wifi_ssid.setText(self.wifiSSID)
        mymw.ui.wifi_ip.setText(self.robot.wifiIP)
        mymw.ui.wifi_mac.setText(self.robot.wifiMAC)
        mymw.ui.wifi_port.setReadOnly(True)
        mymw.ui.wifi_ssid.setReadOnly(True)
        mymw.ui.wifi_pw.setReadOnly(True)
        mymw.ui.wifi_apply.setEnabled(False)
        mymw.ui.wifi_ip.setReadOnly(True)
        mymw.ui.wifi_got_ip.setEnabled(True)
        mymw.ui.wifi_port_open.setEnabled(True)
      self.dataWifi = False
      self.lock.release()
    if self.dataClient:
      self.lock.acquire()
      mymw.ui.wifi_client_1.setText(str(self.client1))
      mymw.ui.wifi_client_2.setText(str(self.client2))
      mymw.ui.wifi_client_3.setText(str(self.client3))
      mymw.ui.wifi_client_4.setText(str(self.client4))
      mymw.ui.wifi_client_5.setText(str(self.client5))
      mymw.ui.wifi_good.setText(str(self.wifiGood))
      mymw.ui.wifi_lost.setText(str(self.wifiLost))
      msg = self.wifiGood + self.wifiLost
      if (msg > 0):
        loss = float(self.wifiLost)/float(msg)
        if (self.wifiLoss == 0):
          self.wifiLoss = float(loss)
        else:
          self.wifiLoss = self.wifiLoss * 0.8 + 0.2 * float(loss)
        mymw.ui.wifi_loss.setText("{:.2f} %".format(self.wifiLoss * 100.0))
      self.dataClient = False
      self.lock.release()
    pass
  # 
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      self.lock.release()
  def dataChangedManuallyWifi(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.wifiInEdit = True
      self.lock.release()
  #
  def cancelEdit(self):
    self.inEdit = False;
  # send data as is in edit fields
  def wifiSendData(self):
    # "wifi 1 " +  mymw.ui.wifi_port.text() # + " " + mymw.ui.wifi_ssid.text()
    robot.conWrite("wifi 1 " + mymw.ui.wifi_port.text() + ' "' + mymw.ui.wifi_ssid.text() + '" "' + mymw.ui.wifi_pw.text() + '"')
    self.wifiInEdit = False
    pass
  # Cansel wifi edit
  def wifiGetData(self):
    #robot.conWrite("v1")
    self.wifiInEdit = False
    pass
  # wifi fields edit
  def wifiEdit(self):
    self.lock.acquire()
    self.wifiInEdit = True
    mymw.ui.wifi_port.setReadOnly(False)
    mymw.ui.wifi_ssid.setReadOnly(False)
    mymw.ui.wifi_pw.setReadOnly(False)
    mymw.ui.wifi_apply.setEnabled(True)
    self.lock.release()
    pass
  def wifiSaveMacList(self):
    try:
      fn = "regbot_mac.txt"
      f = open(fn, "w")
      f.write('%% MAC list for robots in regbot.ini file\r\n')
      for rb in robot.info.robots:
        if (rb.robotID == self.robotID):
          # current robot
          if (mymw.ui.wifi_and_save_IP.isChecked()):
            f.write(str(self.robot.name) + " " + str(self.robot.robotID) + " " + self.robot.wifiIP + " " + str(self.robot.wifiMAC) + "\r\n")
          else:
            f.write(str(self.robot.name) + " " + str(self.robot.robotID) + " " + str(self.robot.wifiMAC) + "\r\n")
        elif rb.robotID > 0:
          if (mymw.ui.wifi_and_save_IP.isChecked()):
            f.write(str(rb.name) + " " + str(rb.robotID) + " " + str(rb.wifiIP) + " " + str(rb.wifiMAC) + "\r\n")
          else:
            f.write(str(rb.name) + " " + str(rb.robotID) + " " + str(rb.wifiMAC) + "\r\n")
      f.close()
    except:
      mymw.ui.statusbar.showMessage("Failed to open file " + fn + "!", 3000)
    pass

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////


class URegTurn(object):
  regActive = 0
  regKp = 0.0
  regTauD = 0.0
  regTauI = 0.0
  regAlpha = 0.0
  regIMax = 0.0
  regStepOn = 0
  regStepOff = 0
  regStepVal = 0 # step value in rad/s or 
  regStepVel = 0 # velocity base
  regTurnLeadFwd = True
  regOutMax = 100
  dataRead = False
  stepValueMotorV = 1.0
  stepValueMotorReg = 5.0
  stepValueTurnReg = 1.57
  velBaseMotorV = 3.0
  velBaseMotorReg = 30.0
  inUpdate = False
  inEdit = False
  showRobotData = True
  regulZNumer = [1.0, 0.0]
  regulZDenom = [0.0, 0.0]
  regulZINumer = [1.0, 0.0]
  regulZIDenom = [0.0, 0.0]
  dataReadZ = False
  dataReadZI = False
  lock = threading.RLock()
  #
  about_box = None
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "rgt":
        #regul_turn_use, regul_turn_kp, regul_turn_ti, regul_turn_td, regul_turn_alpha, regul_turn_i_limit,
        #regul_turn_step_time_on, regul_turn_step_time_off, regul_turn_step_val, regul_turn_step_vel, regul_turn_LeadFwd
        self.regActive = int(gg[1],0)
        self.dataRead = True
        self.regKp = float(gg[2])
        self.regTauI = float(gg[3])
        self.regTauD = float(gg[4])
        self.regAlpha = float(gg[5])
        self.regIMax = float(gg[6])
        self.regStepOn = float(gg[7])
        self.regStepOff = float(gg[8])
        self.regStepVal = float(gg[9])
        self.regStepVel = float(gg[10])
        self.regTurnLeadFwd = int(gg[11], 0)
        self.regOutMax = float(gg[12])
      elif gg[0] == "rgtz":
        self.regulZDenom[1] = float(gg[1])
        # self.regulZDenom[2] = float(gg[2])
        self.regulZNumer[0] = float(gg[2])
        self.regulZNumer[1] = float(gg[3])
        # self.regulZNumer[2] = float(gg[5])
        self.dataReadZ = True
      elif gg[0] == "rgtzi":
        self.regulZIDenom[1] = float(gg[1])
        self.regulZINumer[0] = float(gg[2])
        self.regulZINumer[1] = float(gg[3])
        self.dataReadZI = True 
      else:
        used = False
    except:
      print("URegTurn: data read error - failed on a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (not mymw.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      mymw.ui.reg_turn_apply.setEnabled(False)
      mymw.ui.reg_turn_start.setEnabled(False)
      mymw.ui.reg_turn_read.setEnabled(False)
      mymw.ui.reg_turn_edit.setEnabled(True)
    else:
      mymw.ui.reg_turn_apply.setEnabled(self.inEdit)
      mymw.ui.reg_turn_read.setEnabled(self.inEdit)
      mymw.ui.reg_turn_edit.setEnabled(not self.inEdit)
      mymw.ui.reg_turn_start.setEnabled(not self.inEdit)
    if (self.dataRead):
      self.dataRead = False
      self.inUpdate = True
      if (not self.inEdit):
        pass # reset with robot data
        # base speed depend on velocity regulator
        if (self.regActive):
          self.velBaseMotorReg = self.regStepVel
        else:
          self.velBaseMotorV = self.regStepVel
        # step size depend on turn and vel regulator use
        if (mymw.ui.reg_turn_use.isChecked()):
          self.stepValueTurnReg = self.regStepVal
        else:
          if (mymw.ui.reg_vel_use.isChecked()):
            self.stepValueMotorReg = self.regStepVal
          else:
            self.stepValueMotorV = self.regStepVal
        mymw.ui.reg_turn_step_val.setValue(self.regStepVal)
        mymw.ui.reg_turn_use.setChecked(self.regActive)
        mymw.ui.reg_turn_KP.setValue(self.regKp)
        mymw.ui.reg_turn_tau_d.setValue(self.regTauD)
        mymw.ui.reg_turn_tau_i.setValue(self.regTauI)
        mymw.ui.reg_turn_alpha.setValue(self.regAlpha)
        mymw.ui.reg_turn_ilimit.setValue(self.regIMax)
        mymw.ui.reg_turn_out_limit.setValue(self.regOutMax)
        mymw.ui.reg_turn_step_on.setValue(self.regStepOn)
        mymw.ui.reg_turn_step_off.setValue(self.regStepOff)
        mymw.ui.reg_turn_step_vel.setValue(self.regStepVel)
        mymw.ui.reg_turn_LeadFwd.setChecked(self.regTurnLeadFwd)
      self.inUpdate = False
      if (self.dataReadZ):
        self.dataReadZ = False
        if (self.regulZNumer[1] == 0.0):
          mymw.ui.reg_turn_numer.setText("%g" % (self.regulZNumer[0]))
          mymw.ui.reg_turn_denom.setText("1")
        else:
          mymw.ui.reg_turn_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
          mymw.ui.reg_turn_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
      if (self.dataReadZI):
        self.dataReadZI = False
        if (self.regulZINumer[1] == 0.0):
          mymw.ui.reg_turn_numer_2.setText("0")
          mymw.ui.reg_turn_denom_2.setText("1")
        else:
          mymw.ui.reg_turn_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
          mymw.ui.reg_turn_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      #print("turn data changed manuallt - no timer")
      self.inEdit = True
  def configChanged(self):
    self.inUpdate = True
    if (mymw.ui.reg_vel_use.isChecked()):
      mymw.ui.reg_turn_step_vel.setValue(self.velBaseMotorReg)
    else:
      mymw.ui.reg_turn_step_vel.setValue(self.velBaseMotorV)
    # step size depend on turn and vel regulator use
    if (mymw.ui.reg_turn_use.isChecked()):
      mymw.ui.reg_turn_step_val.setValue(self.stepValueTurnReg)
      mymw.ui.reg_turn_step_value_label.setText("Value [rad]")
    else:
      if (mymw.ui.reg_vel_use.isChecked()):
        mymw.ui.reg_turn_step_val.setValue(self.stepValueMotorReg)
        mymw.ui.reg_turn_step_value_label.setText("Value [rad/s]")
      else:
        mymw.ui.reg_turn_step_val.setValue(self.stepValueMotorV)
        mymw.ui.reg_turn_step_value_label.setText("Value [V]")
    self.inUpdate = False
  def regulatorUseClicked(self):
    self.configChanged();
    mymw.turn_paint_space.repaint()
  def regulatorParamRead(self):
    # cancel pressed
    self.inEdit = False
  def stepOrVelValueChanged(self):
    pass # save step and base vel in right config
    if (not self.inUpdate):
      if (mymw.ui.reg_vel_use.isChecked()):
        self.velBaseMotorReg = mymw.ui.reg_turn_step_vel.value()
      else:
        self.velBaseMotorV = mymw.ui.reg_turn_step_vel.value()
      # step size depend on turn and vel regulator use
      if (mymw.ui.reg_turn_use.isChecked()):
        self.stepValueTurnReg = mymw.ui.reg_turn_step_val.value()
      else:
        if (mymw.ui.reg_vel_use.isChecked()):
          self.stepValueMotorReg = mymw.ui.reg_turn_step_val.value()
        else:
          self.stepValueMotorV = mymw.ui.reg_turn_step_val.value()
  def helpbox(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Turn control</span></p>
                <p>
                <b>ROBOT TURN CONTROL</b>
                <i>USE is not ticked:</i><br />
                * turn is open loop (STEP is in Volts (or approximately rad/sec)<br />
                <i>USE is ticked:</i><br />
                * turn controller is in closed loop (ref is in radians)<br />
                * TAU_I is entered in seconds.<br />
                * if TAU_I=0, then the I-term is omitted.<br />
                * TAU_D is the lead time constant [sec] and and works together with alpha.<br />
                * if TAU_D=0 then the Lead term is omitted.<br /></p>
                <hr />
                <p><b>STEP</b> is applied from ON TIME to OFF TIME when START is pressed<br />
                open loop:<br />
                * the STEP is applied  in motor input units (Volt or rad/sec)<br />
                * positive is CCV<br />
                closed loop:<br />
                * turn is closed loop, with a controller as specified.<br />
                * The STEP is now in radians <br />
                * positive is CCV<br /></p>
                <hr />
                <p>* <b>VELOCITY</b> is average motor input during turn - zero velocity is rather bad<br />
                Units of motor input is determined in the VELOCITY tab.<br /></p>
                <hr />
                <p>
                When <b>EDIT</b> is pressed settings available for editing<br />
                When <b>APPLY</b> is pressed the new settings are send to the robot<br />
                When <b>START</b> is pressed, then the turn_step (mission 1) is started.<br />
                When <b>CANSEL</b> is pressed settings are copied from the robot<br />
                </p>''');
      #about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////


class URegPosition(object):
  regActive = 0
  regKp = 0.0
  regTauD = 0.0
  regTauI = 0.0
  regAlpha = 0.0
  regIMax = 0.0
  regStepTime = 0
  regStepFrom = 0
  regStepTo = 0 # step value in rad/s or 
  #regStepVel = 0 # velocity base
  regTurnLeadFwd = True
  regOutMax = 100
  dataRead = False
  #stepValueMotorV = 1.0
  #stepValueMotorReg = 5.0
  #stepValueTurnReg = 1.57
  #velBaseMotorV = 3.0
  #velBaseMotorReg = 30.0
  inUpdate = False
  inEdit = False
  showRobotData = True
  regulZNumer = [1.0, 0.0]
  regulZDenom = [0.0, 0.0]
  regulZINumer = [1.0, 0.0]
  regulZIDenom = [0.0, 0.0]
  dataReadZ = False
  dataReadZI = False
  lock = threading.RLock()
  #
  about_box = None
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "rgp":
        #regul_turn_use, regul_turn_kp, regul_turn_ti, regul_turn_td, regul_turn_alpha, regul_turn_i_limit,
        #regul_turn_step_time_on, regul_turn_step_time_off, regul_turn_step_val, regul_turn_step_vel, regul_turn_LeadFwd
        self.regActive = int(gg[1],0)
        self.dataRead = True
        self.regKp = float(gg[2])
        self.regTauI = float(gg[3])
        self.regTauD = float(gg[4])
        self.regAlpha = float(gg[5])
        self.regIMax = float(gg[6])
        self.regStepTime = float(gg[7])
        self.regStepFrom = float(gg[8])
        self.regStepTo = float(gg[9])
        self.regTurnLeadFwd = int(gg[10], 0)
        self.regOutMax = float(gg[11])
      elif gg[0] == "rgpz":
        self.regulZDenom[1] = float(gg[1])
        # self.regulZDenom[2] = float(gg[2])
        self.regulZNumer[0] = float(gg[2])
        self.regulZNumer[1] = float(gg[3])
        # self.regulZNumer[2] = float(gg[5])
        self.dataReadZ = True
      elif gg[0] == "rgpzi":
        self.regulZIDenom[1] = float(gg[1])
        self.regulZINumer[0] = float(gg[2])
        self.regulZINumer[1] = float(gg[3])
        self.dataReadZI = True 
      else:
        used = False
    except:
      print("URegPosition: data read error - failed on a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (not mymw.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      mymw.ui.reg_pos_apply.setEnabled(False)
      mymw.ui.reg_pos_start.setEnabled(False)
      mymw.ui.reg_pos_cancel.setEnabled(False)
      mymw.ui.reg_pos_edit.setEnabled(True)
    else:
      mymw.ui.reg_pos_apply.setEnabled(self.inEdit)
      mymw.ui.reg_pos_cancel.setEnabled(self.inEdit)
      mymw.ui.reg_pos_edit.setEnabled(not self.inEdit)
      mymw.ui.reg_pos_start.setEnabled(not self.inEdit)
    if (self.dataRead):
      self.dataRead = False
      self.inUpdate = True
      if (not self.inEdit):
        pass # reset with robot data
        # base speed depend on velocity regulator
        mymw.ui.reg_pos_use.setChecked(self.regActive)
        mymw.ui.reg_pos_KP.setValue(self.regKp)
        mymw.ui.reg_pos_tau_d.setValue(self.regTauD)
        mymw.ui.reg_pos_tau_i.setValue(self.regTauI)
        mymw.ui.reg_pos_alpha.setValue(self.regAlpha)
        mymw.ui.reg_pos_ilimit.setValue(self.regIMax)
        mymw.ui.reg_pos_out_limit.setValue(self.regOutMax)
        mymw.ui.reg_pos_step_time.setValue(self.regStepTime)
        mymw.ui.reg_pos_step_from.setValue(self.regStepFrom)
        mymw.ui.reg_pos_step_to.setValue(self.regStepTo)
        mymw.ui.reg_pos_LeadFwd.setChecked(self.regTurnLeadFwd)
      self.inUpdate = False
      if (self.dataReadZ):
        self.dataReadZ = False
        if (self.regulZNumer[1] == 0.0):
          mymw.ui.reg_pos_numer.setText("%g" % (self.regulZNumer[0]))
          mymw.ui.reg_pos_denom.setText("1")
        else:
          mymw.ui.reg_pos_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
          mymw.ui.reg_pos_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
      if (self.dataReadZI):
        self.dataReadZI = False
        if (self.regulZINumer[1] == 0.0):
          mymw.ui.reg_pos_numer_2.setText("0")
          mymw.ui.reg_pos_denom_2.setText("1")
        else:
          mymw.ui.reg_pos_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
          mymw.ui.reg_pos_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      #print("turn data changed manuallt - not in timed update")
      self.inEdit = True
  def configChanged(self):
    self.inUpdate = True
    pass
    self.inUpdate = False
  def regulatorUseClicked(self):
    self.configChanged();
    mymw.pos_paint_space.repaint()
  def regulatorParamCancel(self):
    # cancel pressed
    self.inEdit = False
  #def stepOrVelValueChanged(self):
    #pass # save step and base vel in right config
    #if (not self.inUpdate):
      #pass
  def helpbox(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Turn control</span></p>
                <p>
                <b>ROBOT POSITION CONTROL</b>
                <i>USE is not ticked:</i><br />
                * Position control is not active.<br />
                <i>USE is ticked:</i><br />
                * position controller is in closed loop (ref is in meters)<br />
                * TAU_I is entered in seconds.<br />
                * if TAU_I=0, then the I-term is omitted.<br />
                * TAU_D is the lead time constant [sec] and and works together with alpha.<br />
                * if TAU_D=0 then the Lead term is omitted.<br /></p>
                <hr />
                <p><b>STEP</b> is applied at ON TIME and change at this time to the "to" value<br />
                <hr />
                <p>
                When <b>EDIT</b> is pressed settings available for editing<br />
                When <b>APPLY</b> is pressed the new settings are send to the robot<br />
                When <b>START</b> is pressed, then the turn_step (mission 1) is started.<br />
                When <b>CANSEL</b> is pressed settings are copied from the robot<br />
                </p>''');
      #about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()
  def regStart(self):
    # set mission to 1 and start
    robot.conWrite("M=7\n")
    robot.conWrite("start\n")
    robot.mainStatus += " mission 7 started\n"
    robot.mainStatusSet = True
  def regulatorParamApply(self):
    robot.conWrite("rp=%d %g %g %g %g %g %g %g %g %d %g\n" % (
        mymw.ui.reg_pos_use.isChecked(), 
        mymw.ui.reg_pos_KP.value(), 
        mymw.ui.reg_pos_tau_i.value(), 
        mymw.ui.reg_pos_tau_d.value(), 
        mymw.ui.reg_pos_alpha.value(),
        mymw.ui.reg_pos_ilimit.value(),
        mymw.ui.reg_pos_step_time.value(),
        mymw.ui.reg_pos_step_from.value(),
        mymw.ui.reg_pos_step_to.value(),
        mymw.ui.reg_pos_LeadFwd.isChecked(),
        mymw.ui.reg_pos_out_limit.value()
        ))
    #mymw.ui.reg_turn_apply.setEnabled(False)
    #self.regTurn.stepOrVelValueChanged()
    self.inEdit = False

  
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class URegVel(object):
  regActive = 0
  regKp = 0.0
  regTauD = 0.0
  regTauI = 0.0
  regAlpha = 0.0
  regIMax = 0.0
  regStepTime = 0.0
  regStepFrom = 0.0
  regStepTo = 0.0
  dataRead = False
  dataReadZ = False
  dataReadZI = False
  regulZNumer = [1.0, 0.0]
  regulZDenom = [0.0, 0.0]
  regulZINumer = [1.0, 0.0]
  regulZIDenom = [0.0, 0.0]
  regVelAccLimit = 100
  regVelEstUse = True
  regVelLeadFwd = True
  regVelVoltLimit = 10 # motor voltage limit [V]
  # if voltage control
  regStepFromV = 2.1
  regStepToV = 4.1
  # if m/s control
  regStepFromReg = 0.20
  regStepToReg = 0.40
  inUpdate = False
  inEdit = False
  #
  about_box= None
  # resource lock
  lock = threading.RLock()
  def readData(self, gg):
    used = True
    self.lock.acquire()
    #print("URegVel " + gg[0] + " length is " + str(len(gg)))
    try: 
      if gg[0] == "rgv":
        self.regActive = int(gg[1],0)
        self.dataRead = True
        self.regKp = float(gg[2])
        self.regTauI = float(gg[3])
        self.regTauD = float(gg[4])
        self.regAlpha = float(gg[5])
        self.regIMax = float(gg[6])
        self.regStepTime = float(gg[7])
        self.regStepFrom = float(gg[8])
        self.regStepTo = float(gg[9])
        if (len(gg) > 10):
          self.regVelAccLimit = float(gg[10])
        if (len(gg) > 11):
          self.regVelEstUse = int(gg[11])
        if (len(gg) > 12):
          self.regVelLeadFwd = int(gg[12])
        if (len(gg) > 13):
          self.regVelVoltLimit = float(gg[13]) 
      elif gg[0] == "rgvz":
        self.regulZDenom[1] = float(gg[1])
        # self.regulZDenom[2] = float(gg[2])
        self.regulZNumer[0] = float(gg[2])
        self.regulZNumer[1] = float(gg[3])
        # self.regulZNumer[2] = float(gg[5])
        self.dataReadZ = True
      elif gg[0] == "rgvzi":
        self.regulZIDenom[1] = float(gg[1])
        self.regulZINumer[0] = float(gg[2])
        self.regulZINumer[1] = float(gg[3])
        self.dataReadZI = True
      else:
        used = False
    except:
      print("URegVel: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (not mymw.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      mymw.ui.reg_vel_apply.setEnabled(False)
      mymw.ui.reg_vel_start.setEnabled(False)
      mymw.ui.reg_vel_read.setEnabled(False)
    elif self.inEdit:
      mymw.ui.reg_vel_apply.setEnabled(True)
      mymw.ui.reg_vel_read.setEnabled(True)
    else:
      mymw.ui.reg_vel_start.setEnabled(True)
    if (self.dataRead):
      self.dataRead = False
      self.inUpdate = True
      # base speed depend on velocity regulator
      if (not self.inEdit):
        if (self.regActive):
          self.regStepFromReg = self.regStepFrom
          self.regStepToReg = self.regStepTo
        else:
          self.regStepFromV = self.regStepFrom
          self.regStepToV = self.regStepTo
        # reset with robot data
        mymw.ui.reg_vel_use.setChecked(self.regActive)
        if (self.regActive):
          mymw.ui.reg_turn_vel_base.setText("base [m/s]")
        else:
          mymw.ui.reg_turn_vel_base.setText("base [V]")
        mymw.ui.reg_vel_KP.setValue(self.regKp)
        mymw.ui.reg_vel_tau_d.setValue(self.regTauD)
        mymw.ui.reg_vel_tau_i.setValue(self.regTauI)
        mymw.ui.reg_vel_alpha.setValue(self.regAlpha)
        mymw.ui.reg_vel_integrate_max.setValue(self.regIMax)
        mymw.ui.reg_vel_apply.setEnabled(False)
        mymw.ui.reg_vel_start.setEnabled(True)
        mymw.ui.reg_vel_edit.setEnabled(True)
        mymw.ui.reg_vel_read.setEnabled(False)
        mymw.ui.reg_vel_steptime.setValue(self.regStepTime)
        mymw.ui.reg_vel_step_from.setValue(self.regStepFrom)
        mymw.ui.reg_vel_step_to.setValue(self.regStepTo)
        if (self.regVelAccLimit < 99):
          mymw.ui.vel_acc_limit_use.setChecked(True)
          mymw.ui.reg_vel_acc_limit.setValue(self.regVelAccLimit)
        else:
          mymw.ui.vel_acc_limit_use.setChecked(False)
          mymw.ui.reg_vel_acc_limit.setValue(100)
        mymw.ui.reg_vel_est_use.setChecked(self.regVelEstUse)
        mymw.ui.reg_vel_LeadFwd.setChecked(self.regVelLeadFwd)
        mymw.ui.reg_vel_volt_limit.setValue(self.regVelVoltLimit)
      self.inUpdate = False
    if (self.dataReadZ):
      self.dataReadZ = False
      if (self.regulZNumer[1] == 0.0):
        mymw.ui.reg_vel_numer.setText("%g" % (self.regulZNumer[0]))
        mymw.ui.reg_vel_denom.setText("1")
      else:
        mymw.ui.reg_vel_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
        mymw.ui.reg_vel_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
    if (self.dataReadZI):
      self.dataReadZI = False
      if (self.regulZINumer[1] == 0.0):
        mymw.ui.reg_vel_numer_2.setText("0")
        mymw.ui.reg_vel_denom_2.setText("1")
      else:
        mymw.ui.reg_vel_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
        mymw.ui.reg_vel_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      mymw.ui.reg_vel_apply.setEnabled(True)
      mymw.ui.reg_vel_start.setEnabled(False)
      mymw.ui.reg_vel_edit.setEnabled(False)
      mymw.ui.reg_vel_read.setEnabled(True)
      self.lock.release()
  def configChanged(self):
    pass # load the right value into step from-to widget
    self.inUpdate = True
    if (mymw.ui.reg_vel_use.isChecked()):
      mymw.ui.reg_vel_step_from.setValue(self.regStepFromReg)
      mymw.ui.reg_vel_step_to.setValue(self.regStepToReg)
      mymw.ui.reg_turn_vel_base.setText("base [m/s]")
      mymw.ui.reg_vel_step_label.setText("From [m/s]")
      mymw.ui.reg_vel_step_label_2.setText("To [m/s]")
    else:
      mymw.ui.reg_vel_step_from.setValue(self.regStepFromV)
      mymw.ui.reg_vel_step_to.setValue(self.regStepToV)      
      mymw.ui.reg_turn_vel_base.setText("base [V]")
      mymw.ui.reg_vel_step_label.setText("From [Volt]")
      mymw.ui.reg_vel_step_label_2.setText("To [Volt]")
    self.inUpdate = False
  def regulatorUseClicked(self):
    robot.regTurn.configChanged()
    self.configChanged()
    mymw.ui.reg_vel_frame.repaint()
  def regulatorParamRead(self):
    # this is the cancel button
    # just disable edit, then data is from robot
    # mymw.ui.reg_vel_apply.setEnabled(False)
    self.inEdit = False
  def stepValueChanged(self):
    pass # save step and base vel in right config
    if (not self.inUpdate):
      if (mymw.ui.reg_vel_use.isChecked()):
        self.regStepFromReg = mymw.ui.reg_vel_step_from.value()
        self.regStepToReg = mymw.ui.reg_vel_step_to.value()
      else:
        self.regStepFromV = mymw.ui.reg_vel_step_from.value()
        self.regStepToV = mymw.ui.reg_vel_step_to.value()
    #
  def accLimitUse(self):
    self.inUpdate = True
    if (not mymw.ui.vel_acc_limit_use.isChecked()):
      mymw.ui.reg_vel_acc_limit.setValue(100.0)
    else:    
      mymw.ui.reg_vel_acc_limit.setValue(self.regVelAccLimit)
    self.inUpdate = False
  def helpbox(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Motor velocity control</span></p>
                <p>
                <b>MOTOR VEL CONTROL</b> (both motors): <br />
                <i>USE not ticked</i>:<br />
                * Velocity is open loop (STEP is in Volts)<br />
                <i>USE is ticked</i>:<br />
                * Velocity controller is in closed loop (ref is in m/s)<br />
                * <b>Kp</b> is proportional gain<br />
                * <b>TAU_I</b> is entered in seconds.<br />
                * if TAU_I=0, then the I-term is omitted.<br />
                The lead term is either in the forward branch or in the reverse branch, controllerd by <b>Lead in forward</b>.<br />
                * <b>TAU_D</b> is the lead time constant [sec] and works together with <b>alpha</b>.<br />
                * if TAU_D=0 then the Lead term is omitted.<br />
                * <b>I-MAX</b> limits the integration term [V] symmetric +/-, if 0 then no limit.</p>
                <hr />
                <p><b>STEP</b> is the step applied when START is pressed <br />
                open loop:<br />
                * the input <b>STEP FROM</b> and <b>STEP TO</b> is in motor anchor voltage [V]<br />
                voltages below 1.0 volt will in general not start the motors.<br />
                maximum voltage is +/-9V<br />
                closed loop:<br />
                * velocity is closed loop, with a controller as specified by the K_P etc.<br />
                * The STEP FROM and STEP TO is now in rad/s. minimum is in the order of +/- 0.06 m/s, maximum is about +/-1.5 m/s </p>
                <hr />
                <p><b>Edit</b> will stop updating of fieds from robot<br/>
                When <b>Cancel</b> is pressed settings are copied from the robot<br />
                When <b>APPLY</b> is pressed the new settings are implemented on the robot. 
                When <b>START</b> is pressed, then the step (mission 0) is started.
                (Enabled when shown values are applied to the robot).</p>
                <hr />
                <p><b>Other settings</b><br />
                The <b>acceleration limit</b> limits the acceleration by limiting the increase rate in the velocity reference.<br /> 
                When using balance control this acceleration limit is used for the mission velocity.<br />
                The <b>velocity estimator</b> uses the change in anchor voltage to estimate a velocity change and merges 
                this with the encoder velocity estimate in a complementary filter, where the crossover frequency depends on the actual velocity.
                This has no effect if the velocity is above ~10cm/sec.<br />
                <b>Motor voltage limit</b> limits the maximum voltage the motor will ever see (limits motor PWM).
                </p>''');
      #about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class URegBal(object):
  # balance control
  regActive = 0
  regKp = 0.0
  regTauD = 0.0
  regTauI = 0.0
  regAlpha = 0.0
  regIMax = 0.0
  regLeadFwd = True
  regOutMax = 101
  regLeadGyro = True
  # mission velocity control
  regMvActive = 0
  regMvKp = 0.0
  regMvTauD = 0.0
  regMvTauI = 0.0
  regMvAlpha = 0.0
  regMvZeta = 0.0
  regMvIMax = 0.0
  regMvUse = False
  regMvLeadFwd = True
  regMvOutMax = 102
  # step
  regStepTime = 0.0
  regStepFrom = 0.0
  regStepTo = 0.0
  regStepVFrom = 0.0
  regStepVTo = 0.0
  regStepMSFrom = 0.0
  regStepMSTo = 0.0
  regStepMVFrom = 0.0
  regStepMVTo = 0.0
  # management
  dataRead = False
  dataReadMV = False
  dataReadZ = False
  dataReadZI = False
  dataReadMvZ = False
  dataReadMvZI = False
  # z-domain values
  regulZNumer = [1.0, 0.0]
  regulZDenom = [0.0, 0.0]
  regulZINumer = [1.0, 0.0]
  regulZIDenom = [0.0, 0.0]
  regulMvZNumer = [1.0, 0.0, 0.0]
  regulMvZDenom = [0.0, 0.0, 0.0]
  regulMvZINumer = [1.0, 0.0]
  regulMvZIDenom = [0.0, 0.0]
  inUpdate = False
  inEdit = False
  #
  about_box = None
  # resource lock
  lock = threading.RLock()
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "rgb":
        self.regActive = int(gg[1],0)
        self.dataRead = True
        self.regKp = float(gg[2])
        self.regTauI = float(gg[3])
        self.regTauD = float(gg[4])
        self.regAlpha = float(gg[5])
        self.regIMax = float(gg[6])
        self.regStepTime = float(gg[7])
        self.regMvUse = int(gg[10],0)
        if (self.regMvUse):
          self.regStepVFrom = float(gg[8])
          self.regStepVTo = float(gg[9])
        elif (self.regActive):
          self.regStepFrom = float(gg[8])
          self.regStepTo = float(gg[9])
        else:
          self.regStepMSFrom = float(gg[8])
          self.regStepMSTo = float(gg[9])
        self.regLeadFwd = int(gg[11], 0)
        self.regOutMax = float(gg[12])
        self.regLeadGyro = int(gg[13], 0)
      elif gg[0] == "rgbz":
        self.regulZDenom[1] = float(gg[1])
        # self.regulZDenom[2] = float(gg[2])
        self.regulZNumer[0] = float(gg[2])
        self.regulZNumer[1] = float(gg[3])
        # self.regulZNumer[2] = float(gg[5])
        self.dataReadZ = True
      elif gg[0] == "rgbzi":
        self.regulZIDenom[1] = float(gg[1])
        self.regulZINumer[0] = float(gg[2])
        self.regulZINumer[1] = float(gg[3])
        self.dataReadZI = True
      elif gg[0] == "rgmv":
        self.dataReadMV = True
        self.regMvKp = float(gg[1])
        self.regMvTauI = float(gg[2])
        self.regMvTauD = float(gg[3])
        self.regMvAlpha = float(gg[4])
        self.regMvZeta = float(gg[5])
        self.regMvIMax = float(gg[6])
        self.regMvLeadFwd = int(gg[7],0)
        self.regMvOutMax = float(gg[8])
      elif gg[0] == "rgmvz":
        self.regulMvZDenom[1] = float(gg[1])
        self.regulMvZDenom[2] = float(gg[2])
        self.regulMvZNumer[0] = float(gg[3])
        self.regulMvZNumer[1] = float(gg[4])
        self.regulMvZNumer[2] = float(gg[5])
        self.dataReadMvZ = True
      elif gg[0] == "rgmvzi":
        self.regulMvZIDenom[1] = float(gg[1])
        self.regulMvZINumer[0] = float(gg[2])
        self.regulMvZINumer[1] = float(gg[3])
        self.dataReadMvZI = True
      else:
        used = False
    except:
      print("URegBal: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (not mymw.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      mymw.ui.reg_bal_apply.setEnabled(False)
      mymw.ui.reg_bal_start.setEnabled(False)
      mymw.ui.reg_bal_read.setEnabled(False) # cancel
      mymw.ui.reg_bal_edit.setEnabled(not self.inEdit)
    else:
      mymw.ui.reg_bal_start.setEnabled(not self.inEdit)
      mymw.ui.reg_bal_apply.setEnabled(self.inEdit)
      mymw.ui.reg_bal_read.setEnabled(self.inEdit) # cancel
      mymw.ui.reg_bal_edit.setEnabled(not self.inEdit)
    mymw.ui.reg_bal_LeadFwd.setEnabled(not mymw.ui.reg_bal_LeadGyro.isChecked())
    if (self.dataRead and self.dataReadMV):
      self.dataRead = False
      self.dataReadMV = False
      self.inUpdate = True
      # base speed depend on velocity regulator
      if (not self.inEdit):
        # reset with robot data
        mymw.ui.reg_bal_use.setChecked(self.regActive)
        mymw.ui.reg_bal_KP.setValue(self.regKp)
        mymw.ui.reg_bal_tau_d.setValue(self.regTauD)
        mymw.ui.reg_bal_tau_i.setValue(self.regTauI)
        mymw.ui.reg_bal_alpha.setValue(self.regAlpha)
        mymw.ui.reg_bal_integrate_max.setValue(self.regIMax)
        mymw.ui.reg_bal_out_limit.setValue(self.regOutMax)
        if (self.regOutMax < 0.5):
          mymw.ui.label_out_limit.setStyleSheet("QLabel { background-color : red; }")
        else:
          mymw.ui.label_out_limit.setStyleSheet("QLabel { background-color : lightGray; }")
        mymw.ui.reg_bal_steptime.setValue(self.regStepTime)
        mymw.ui.reg_bal_LeadFwd.setChecked(self.regLeadFwd)
        mymw.ui.reg_bal_LeadGyro.setChecked(self.regLeadGyro)
        mymw.ui.reg_bal_alpha.setEnabled(not self.regLeadGyro)
        #print("show data bal V:" + str(self.regStepVFrom) + "-" + str(self.regStepVTo) + ", B: " + str(self.regStepFrom) + "-" + str(self.regStepTo))
        if (self.regMvUse):
          mymw.ui.reg_bal_step_from.setValue(self.regStepVFrom)
          mymw.ui.reg_bal_step_to.setValue(self.regStepVTo)
        else:
          mymw.ui.reg_bal_step_from.setValue(self.regStepFrom)
          mymw.ui.reg_bal_step_to.setValue(self.regStepTo)
        # mission velocity ctrl
        mymw.ui.reg_mvel_KP.setValue(self.regMvKp)
        mymw.ui.reg_mvel_tau_d.setValue(self.regMvTauD)
        mymw.ui.reg_mvel_tau_i.setValue(self.regMvTauI)
        mymw.ui.reg_mvel_alpha.setValue(self.regMvAlpha)
        mymw.ui.reg_mvel_zeta.setValue(self.regMvZeta)
        mymw.ui.reg_mvel_integrate_max.setValue(self.regMvIMax)
        mymw.ui.reg_balvel_use.setChecked(self.regMvUse)
        mymw.ui.reg_mvel_LeadFwd.setChecked(self.regMvLeadFwd)
        mymw.ui.reg_mvel_out_limit.setValue(self.regMvOutMax)
      self.inUpdate = False
    if (self.dataReadZ):
      self.dataReadZ = False
      if (self.regulZNumer[1] == 0.0):
        mymw.ui.reg_bal_numer.setText("%g" % (self.regulZNumer[0]))
        mymw.ui.reg_bal_denom.setText("1")
      else:
        mymw.ui.reg_bal_numer.setText("%g + %g z^-1" % (self.regulZNumer[0], self.regulZNumer[1]))
        mymw.ui.reg_bal_denom.setText( "1 - %g z^-1" % (-self.regulZDenom[1]))
    if (self.dataReadZI):
      self.dataReadZI = False
      if (self.regulZINumer[1] == 0.0):
        mymw.ui.reg_bali_numer.setText("0")
        mymw.ui.reg_bali_denom.setText("1")
      else:
        mymw.ui.reg_bali_numer.setText("%g - %g z^-1" % (self.regulZINumer[0], -self.regulZINumer[1]))
        mymw.ui.reg_bali_denom.setText( "1 - %g z^-1" % (-self.regulZIDenom[1]))
    if (self.dataReadMvZ):
      self.dataReadMvZ = False
      if (self.regulMvZNumer[1] == 0.0):
        mymw.ui.reg_bal_mv_numer.setText("%g" % (self.regulMvZNumer[0]))
        mymw.ui.reg_bal_mv_denom.setText("1")
      else:
        mymw.ui.reg_bal_mv_numer.setText("%g + %g z^-1 - %g z^-2" % (self.regulMvZNumer[0], self.regulMvZNumer[1], -self.regulMvZNumer[2]))
        mymw.ui.reg_bal_mv_denom.setText( "1 - %g z^-1 + %g z^-2" % (-self.regulMvZDenom[1], self.regulMvZDenom[2]))
    if (self.dataReadMvZI):
      self.dataReadMvZI = False
      if (self.regulMvZINumer[1] == 0.0):
        mymw.ui.reg_bal_mv_numer_2.setText("0")
        mymw.ui.reg_bal_mv_denom_2.setText("1")
      else:
        mymw.ui.reg_bal_mv_numer_2.setText("%g  -  %g z^-1" % (self.regulMvZINumer[0], -self.regulMvZINumer[1]))
        mymw.ui.reg_bal_mv_denom_2.setText( "1  -  %g z^-1" % (-self.regulMvZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      if ( mymw.ui.reg_bal_out_limit.value() < 0.5):
        mymw.ui.label_out_limit.setStyleSheet("QLabel { background-color : red; }")
      else:
        mymw.ui.label_out_limit.setStyleSheet("QLabel { background-color : lightGray; }")
      if (mymw.ui.reg_balvel_use.isChecked()):
        mymw.ui.reg_bal_step_from.setValue(self.regStepVFrom)
        mymw.ui.reg_bal_step_to.setValue(self.regStepVTo)
      else:
        mymw.ui.reg_bal_step_from.setValue(self.regStepFrom)
        mymw.ui.reg_bal_step_to.setValue(self.regStepTo)
      self.lock.release()
  def regulatorUseClicked(self):
    self.configChanged()
    #mymw.ui.bal_regul_frame.repaint()
  def configChanged(self):
    # load the right value into step from-to widget
    #print("config changed bal V:" + str(self.regStepVFrom) + "-" + str(self.regStepVTo) + ", B: " + str(self.regStepFrom) + "-" + str(self.regStepTo))
    self.inUpdate = True
    if (mymw.ui.reg_balvel_use.isChecked()):
      mymw.ui.label_reg_balvel_step_from.setText("step from [m/s]")
      mymw.ui.label_reg_balvel_step_to.setText("step to [m/s]")
      mymw.ui.reg_bal_step_from.setValue(self.regStepVFrom)
      mymw.ui.reg_bal_step_to.setValue(self.regStepVTo)
    elif (mymw.ui.reg_bal_use.isChecked()):
      mymw.ui.label_reg_balvel_step_from.setText("step from [rad]")
      mymw.ui.label_reg_balvel_step_to.setText("step to [rad]")
      mymw.ui.reg_bal_step_from.setValue(self.regStepFrom)
      mymw.ui.reg_bal_step_to.setValue(self.regStepTo)
    elif (mymw.ui.reg_vel_use.isChecked()):
      mymw.ui.label_reg_balvel_step_from.setText("step from [m/s]")
      mymw.ui.label_reg_balvel_step_to.setText("step to [m/s]")
      mymw.ui.reg_bal_step_from.setValue(self.regStepMSFrom)
      mymw.ui.reg_bal_step_to.setValue(self.regStepMSTo)
    else:
      mymw.ui.label_reg_balvel_step_from.setText("step from [Volt]")
      mymw.ui.label_reg_balvel_step_to.setText("step to [Volt]")
      mymw.ui.reg_bal_step_from.setValue(self.regStepMVFrom)
      mymw.ui.reg_bal_step_to.setValue(self.regStepMVTo)
    self.inUpdate = False
    mymw.ui.bal_regul_frame.repaint()
  def regulatorParamRead(self):
    # cancel button pressed
    # just disable the apply buttons, then data is from robot
    self.inEdit = False
    #mymw.ui.reg_bal_apply.setEnabled(False)
    #mymw.ui.reg_mvel_apply.setEnabled(False)
  def stepValueChanged(self):
    pass # save step and base vel in right config
    if (not self.inUpdate):
      if (mymw.ui.reg_balvel_use.isChecked()):
        self.regStepVFrom = mymw.ui.reg_bal_step_from.value()
        self.regStepVTo = mymw.ui.reg_bal_step_to.value()
      else:
        self.regStepFrom = mymw.ui.reg_bal_step_from.value()
        self.regStepTo = mymw.ui.reg_bal_step_to.value()
    #
  def helpbox(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    if self.about_box == None:
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Balance control</span></p>
                <b>TILT angle CONTROL</b>
                Control the tilt angle. This should be zero to keep the balance (or a small value that keep the center of gravity just above the wheels).<br />
                The output of the controller is either the reference to the tilt velocity controller or the input to the drive system.<br />
                <i>USE is not ticked:</i><br />
                * No ballance control<br />
                <i>USE is ticked:</i><br />
                * Balance angleis used (ref is in radians, zero is upright, see also the balance offset in the robot pane)<br />
                * parameters are as for any PI-Lead controller<br /></p>
                <hr />
                <p><b>REF</b> is a fixed reference input used when START is pressed<br />
                <br /></p>
                <hr />
                <p>
                Mission speed control, with the usual parameters.
                It may be a good idea to limit the output 
                </p>
                <hr />
                <p><b>Mission velocity step</b><br />
                Step in mission velocity step with a given velocity before and after the step. 
                The unit of the step value is dependent on the controler receiving the step.
                Different step values are maintained for different configurations (also in save/load).
                </p>
                <p>When <b>EDIT</b> is pressed settings are open for adjustments<br />
                When <b>CANCEL</b> is pressed settings are copied from the robot<br />
                When <b>APPLY</b> is pressed the new settings are send to the robot<br />
                When <b>START</b> is pressed, then the turn_step (mission 1) is started.<br />
                NB! settings will be lost when robot reboots - unless "save on Robot" is pressed.
                </p>''');
      #about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()
    
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////    

class URegSSBal(object):
  # balance control
  regActive = 0
  regKTilt = 0.0
  regKGyro = 0.0
  regKPos = 0.0
  regKVel = 0.0
  regKMotor = 0.0
  regOutMax = 101
  regLeadGyro = True
  # step
  regStepTime = 0.0
  regStepFrom = 0.0
  regStepTo = 0.0
  regStepPos = 0;
  # management
  dataRead = False
  inUpdate = False
  # help
  about_box = None
  # resource lock
  lock = threading.RLock()
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "rgs":
        self.regActive = int(gg[1],0)
        self.dataRead = True
        self.regKTilt = float(gg[2])
        self.regKGyro = float(gg[3])
        self.regKPos = float(gg[4])
        self.regKVel = float(gg[5])
        self.regKMotor = float(gg[6])
        self.regOutMax = float(gg[7])
        self.regStepTime = float(gg[8])
        self.regStepFrom = float(gg[9])
        self.regStepTo = float(gg[10])
        self.regStepPos = int(gg[11], 0)
      else:
        used = False
    except:
      print("URegSSBal: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (self.dataRead):
      self.dataRead = False
      self.inUpdate = True
      # base speed depend on velocity regulator
      if (not mymw.ui.reg_ss_apply.isEnabled()):
        # reset with robot data
        mymw.ui.reg_bal_ss_use.setChecked(self.regActive)
        mymw.ui.reg_bal_ss_k_tilt.setValue(self.regKTilt)
        mymw.ui.reg_bal_ss_k_gyro.setValue(self.regKGyro)
        mymw.ui.reg_bal_ss_k_pos.setValue(self.regKPos)
        mymw.ui.reg_bal_ss_k_vel.setValue(self.regKVel)
        mymw.ui.reg_bal_ss_k_motor.setValue(self.regKMotor)
        mymw.ui.reg_bal_ss_out_limit.setValue(self.regOutMax)
        if (self.regOutMax < 15):
          mymw.ui.label_ss_limit.setStyleSheet("QLabel { background-color : red; }")
        else:
          mymw.ui.label_ss_limit.setStyleSheet("QLabel { background-color : default; }")
        mymw.ui.reg_bal_ss_steptime.setValue(self.regStepTime)
        mymw.ui.reg_bal_ss_step_from.setValue(self.regStepFrom)
        mymw.ui.reg_bal_ss_step_to.setValue(self.regStepTo)
        #mymw.ui.reg_bal_ss_step.setCurrentIndex(self.regStepPos)
        mymw.ui.reg_bal_ss_step.setValue(self.regStepPos)
        if (self.regStepPos == 0):
          mymw.ui.label_reg_bal_ss_from.setText("From [m]")
          mymw.ui.label_reg_bal_ss_to.setText("To [m]")
          mymw.ui.label_reg_bal_ss_to_3.setText("to input position")
        elif (self.regStepPos == 1):
          mymw.ui.label_reg_bal_ss_from.setText("From [m/s]")
          mymw.ui.label_reg_bal_ss_to.setText("To [m/s]")
          mymw.ui.label_reg_bal_ss_to_3.setText("to input velocity")
        else:
          mymw.ui.label_reg_bal_ss_from.setText("From [rad/s]")
          mymw.ui.label_reg_bal_ss_to.setText("To [rad/s]")
          mymw.ui.label_reg_bal_ss_to_3.setText("to input 2?")
        # enable the right buttons
        mymw.ui.reg_ss_read.setEnabled(False)
        mymw.ui.reg_ss_start.setEnabled(True)
        mymw.ui.reg_ss_edit.setEnabled(True)
        mymw.ui.reg_ss_apply.setEnabled(False)
        mymw.ui.frame_ss_ctrl.setEnabled(self.regActive)
      self.inUpdate = False
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      mymw.ui.reg_ss_apply.setEnabled(True)
      mymw.ui.reg_ss_read.setEnabled(True)
      mymw.ui.reg_ss_start.setEnabled(False)
      mymw.ui.reg_ss_edit.setEnabled(False)
      if (mymw.ui.reg_bal_ss_step.value() == 0):
        mymw.ui.label_reg_bal_ss_from.setText("From [m]")
        mymw.ui.label_reg_bal_ss_to.setText("To [m]")
        mymw.ui.label_reg_bal_ss_to_3.setText("to input position")
      elif (mymw.ui.reg_bal_ss_step.value() == 1):
        mymw.ui.label_reg_bal_ss_from.setText("From [m/s]")
        mymw.ui.label_reg_bal_ss_to.setText("To [m/s]")
        mymw.ui.label_reg_bal_ss_to_3.setText("to input velocity")
      else:
        mymw.ui.label_reg_bal_ss_from.setText("From [rad/s]")
        mymw.ui.label_reg_bal_ss_to.setText("To [rad/s]")
        mymw.ui.label_reg_bal_ss_to_3.setText("to input 2?")
      if (mymw.ui.reg_bal_ss_out_limit.value() < 15):
        mymw.ui.label_ss_limit.setStyleSheet("QLabel { background-color : red; }")
      else:
        mymw.ui.label_ss_limit.setStyleSheet("QLabel { background-color : lightGray; }")
      self.lock.release()
  def regulatorUseClicked(self):
    self.configChanged()
    #mymw.ui.bal_regul_frame.repaint()
  def configChanged(self):
    # load the right value into step from-to widget
    #print("config changed bal V:" + str(self.regStepVFrom) + "-" + str(self.regStepVTo) + ", B: " + str(self.regStepFrom) + "-" + str(self.regStepTo))
    self.inUpdate = True
    if (mymw.ui.reg_bal_ss_use.isChecked()):
      mymw.ui.frame_ss_ctrl.setEnabled(True)
    else:
      mymw.ui.frame_ss_ctrl.setEnabled(False)
    self.inUpdate = False
    mymw.ui.bal_ss_frame.repaint()
  def regulatorParamRead(self):
    # just disable the apply buttons, then data is from robot
    mymw.ui.reg_ss_apply.setEnabled(False)
  def stepValueChanged(self):
    pass # save step and base vel in right config
    #if (not self.inUpdate):
      #if (mymw.ui.reg_balvel_use.isChecked()):
        #self.regStepVFrom = mymw.ui.reg_bal_step_from.value()
        #self.regStepVTo = mymw.ui.reg_bal_step_to.value()
      #else:
        #self.regStepFrom = mymw.ui.reg_bal_step_from.value()
        #self.regStepTo = mymw.ui.reg_bal_step_to.value()
    #
  def helpbox(self):
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Balance control State space method</span></p>
                <p>
                <b>Balance CONTROL</b>
                Control the tilt angle. This should be zero to keep the balance (or a small value that keep the center of gravity just above the wheels).<br />
                The output of the controller is either the reference to the tilt velocity controller or the input to the drive system.<br />
                <i>USE is not ticked:</i><br />
                * Balance is controlled as specified in the balance page<br />
                <i>USE is ticked:</i><br />
                Balance is controled from here.<br />
                * Tilt angle is in radians 0 is uptight - offset in Robot page).<br />
                * Gyro is in rad/s (bias calibration in IMU page).<br />
                * Position is in meters (zero when start is pressed).<br />
                * Velocity is in meter/sec.<br />
                * Motor is in wheel rad/sec (reference to motor velocity controller).<br />
                * Output is in wheel rotation velocity  rad/sec.<br />
                * Out limit is in wheel rotation velocity rad/sec.<br />
                <hr />
                <p>When <b>EDIT</b> is pressed settings are open for adjustments<br />
                When <b>CANCEL</b> is pressed settings are copied from the robot<br />
                When <b>APPLY</b> is pressed the new settings are send to the robot<br />
                When <b>START</b> is pressed, then the turn_step (mission 1) is started.<br />
                NB! settings will be lost when robot reboots - unless "save on Robot" is pressed.
                </p>''');
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()
    
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////    


    
class ULineSensor(object):
  # line sensor
  lineValue = [0,0,0,0,0,0,0,0]
  lineMaxWhite = [0,0,0,0,0,0,0,0]
  lineMaxBlack = [0,0,0,0,0,0,0,0]
  lineWhite = False
  lineUse = True
  edgeLeft = 0.0
  edgeRight = 0.0
  edgeLeftValid = False
  edgeRightValid = False
  followLeft = True
  # crossing
  crossingWhite = False
  crossingBlack = False
  crossingWhiteCnt = 0
  crossingBlackCnt = 0
  power_high = False
  power_auto = True
  # management
  dataReadLiv = False
  dataReadLip = False
  dataReadCtrl = False
  dataReadZ = False
  dataReadZI = False
  dataReadbw = True
  #
  inUpdate = False
  inEdit = False
  # 
  #regActive = True
  regKp = 0.0
  regTauD = 0.0
  regTauI = 0.0
  regAlpha = 0.0
  regIMax = 0.0
  regStepOn = 0
  regStepFrom = 0
  regStepTo = 0 # step value in rad/s or 
  regStepVel = 0 # velocity base
  regTurnLeadFwd = True
  regOutMax = 100
  #
  regulZNumer = [1.0, 0.0]
  regulZDenom = [0.0, 0.0]
  regulZINumer = [1.0, 0.0]
  regulZIDenom = [0.0, 0.0]
  #
  about_box = None
  # resource lock
  lock = threading.RLock()
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "liv":
        if mymw.ui.ls_show_normalized.isChecked():
          mv = mymw.ui.line_disp_max_value.value()
          self.lineValue[0] = mv * (int(gg[1],0) - self.lineMaxBlack[0])/(self.lineMaxWhite[0] - self.lineMaxBlack[0])
          self.lineValue[1] = mv * (int(gg[2],0) - self.lineMaxBlack[1])/(self.lineMaxWhite[1] - self.lineMaxBlack[1])
          self.lineValue[2] = mv * (int(gg[3],0) - self.lineMaxBlack[2])/(self.lineMaxWhite[2] - self.lineMaxBlack[2])
          self.lineValue[3] = mv * (int(gg[4],0) - self.lineMaxBlack[3])/(self.lineMaxWhite[3] - self.lineMaxBlack[3])
          self.lineValue[4] = mv * (int(gg[5],0) - self.lineMaxBlack[4])/(self.lineMaxWhite[4] - self.lineMaxBlack[4])
          self.lineValue[5] = mv * (int(gg[6],0) - self.lineMaxBlack[5])/(self.lineMaxWhite[5] - self.lineMaxBlack[5])
          self.lineValue[6] = mv * (int(gg[7],0) - self.lineMaxBlack[6])/(self.lineMaxWhite[6] - self.lineMaxBlack[6])
          self.lineValue[7] = mv * (int(gg[8],0) - self.lineMaxBlack[7])/(self.lineMaxWhite[7] - self.lineMaxBlack[7])
        else: # show raw values
          self.lineValue[0] = int(gg[1],0)
          self.lineValue[1] = int(gg[2],0)
          self.lineValue[2] = int(gg[3],0)
          self.lineValue[3] = int(gg[4],0)
          self.lineValue[4] = int(gg[5],0)
          self.lineValue[5] = int(gg[6],0)
          self.lineValue[6] = int(gg[7],0)
          self.lineValue[7] = int(gg[8],0)
          pass
        self.dataReadLiv = True
      elif gg[0] == "liw":
        self.lineMaxWhite[0] = int(gg[1],0)
        self.lineMaxWhite[1] = int(gg[2],0)
        self.lineMaxWhite[2] = int(gg[3],0)
        self.lineMaxWhite[3] = int(gg[4],0)
        self.lineMaxWhite[4] = int(gg[5],0)
        self.lineMaxWhite[5] = int(gg[6],0)
        self.lineMaxWhite[6] = int(gg[7],0)
        self.lineMaxWhite[7] = int(gg[8],0)
        self.dataReadbw = True
      elif gg[0] == "lib":
        self.lineMaxBlack[0] = int(gg[1],0)
        self.lineMaxBlack[1] = int(gg[2],0)
        self.lineMaxBlack[2] = int(gg[3],0)
        self.lineMaxBlack[3] = int(gg[4],0)
        self.lineMaxBlack[4] = int(gg[5],0)
        self.lineMaxBlack[5] = int(gg[6],0)
        self.lineMaxBlack[6] = int(gg[7],0)
        self.lineMaxBlack[7] = int(gg[8],0)
        self.dataReadbw = True
      elif gg[0] == "lip":
        self.lineUse = int(gg[1],0)
        self.lineWhite = int(gg[2],0)
        self.edgeLeft = float(gg[3])
        self.edgeLeftValid = int(gg[4],0)
        self.edgeRight = float(gg[5])
        self.edgeRightValid = int(gg[6],0)
        self.followLeft = int(gg[7],0)
        self.crossingWhite = int(gg[8],0)
        self.crossingBlack = int(gg[9],0)
        self.crossingWhiteCnt = int(gg[10],0)
        self.crossingBlackCnt = int(gg[11],0)
        self.power_high = int(gg[12],0)
        self.power_auto = int(gg[13],0)
        self.dataReadLip = True
      elif gg[0] == "rgl":
        #print("# got RGL")
        #regul_turn_use, regul_turn_kp, regul_turn_ti, regul_turn_td, regul_turn_alpha, regul_turn_i_limit,
        #regul_turn_step_time_on, regul_turn_step_time_off, regul_turn_step_val, regul_turn_step_vel, regul_turn_LeadFwd
        #self.regActive = int(gg[1],0)
        self.dataReadCtrl = True
        self.regKp = float(gg[2])
        self.regTauI = float(gg[3])
        self.regTauD = float(gg[4])
        self.regAlpha = float(gg[5])
        self.regIMax = float(gg[6])
        self.regStepOn = float(gg[7])
        self.regStepFrom= float(gg[8])
        self.regStepTo = float(gg[9])
        self.regStepVel = float(gg[10])
        self.regTurnLeadFwd = int(gg[11], 0)
        self.regOutMax = float(gg[12])
      elif gg[0] == "rglz":
        self.regulZDenom[1] = float(gg[1])
        # self.regulZDenom[2] = float(gg[2])
        self.regulZNumer[0] = float(gg[2])
        self.regulZNumer[1] = float(gg[3])
        # self.regulZNumer[2] = float(gg[5])
        self.dataReadZ = True
      elif gg[0] == "rglzi":
        self.regulZIDenom[1] = float(gg[1])
        self.regulZINumer[0] = float(gg[2])
        self.regulZINumer[1] = float(gg[3])
        self.dataReadZI = True 
      else:
        used = False
    except:
      print("URegLine: data read error - skipped a " + gg[0] + " length=" + str(len(gg)))
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (not mymw.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      mymw.ui.reg_ls_apply.setEnabled(False)
      mymw.ui.reg_ls_start.setEnabled(False)
      mymw.ui.reg_ls_cancel.setEnabled(False) # cancel
      mymw.ui.reg_ls_edit.setEnabled(not self.inEdit)
    else:
      mymw.ui.reg_ls_start.setEnabled(not self.inEdit)
      mymw.ui.reg_ls_apply.setEnabled(self.inEdit)
      mymw.ui.reg_ls_cancel.setEnabled(self.inEdit) # cancel
      mymw.ui.reg_ls_edit.setEnabled(not self.inEdit)
    if self.dataReadbw:
      # calibration settings
      self.dataReadbw = False
      mymw.ui.ls_max_white_1.setText(str(self.lineMaxWhite[0]))
      mymw.ui.ls_max_white_2.setText(str(self.lineMaxWhite[1]))
      mymw.ui.ls_max_white_3.setText(str(self.lineMaxWhite[2]))
      mymw.ui.ls_max_white_4.setText(str(self.lineMaxWhite[3]))
      mymw.ui.ls_max_white_5.setText(str(self.lineMaxWhite[4]))
      mymw.ui.ls_max_white_6.setText(str(self.lineMaxWhite[5]))
      mymw.ui.ls_max_white_7.setText(str(self.lineMaxWhite[6]))
      mymw.ui.ls_max_white_8.setText(str(self.lineMaxWhite[7]))
      mymw.ui.ls_max_black_1.setText(str(self.lineMaxBlack[0]))
      mymw.ui.ls_max_black_2.setText(str(self.lineMaxBlack[1]))
      mymw.ui.ls_max_black_3.setText(str(self.lineMaxBlack[2]))
      mymw.ui.ls_max_black_4.setText(str(self.lineMaxBlack[3]))
      mymw.ui.ls_max_black_5.setText(str(self.lineMaxBlack[4]))
      mymw.ui.ls_max_black_6.setText(str(self.lineMaxBlack[5]))
      mymw.ui.ls_max_black_7.setText(str(self.lineMaxBlack[6]))
      mymw.ui.ls_max_black_8.setText(str(self.lineMaxBlack[7]))
    if not self.inEdit:
      # Show new flags and result
      if (self.dataReadLip):
        self.dataReadLip = False
        self.inUpdate = True
        mymw.ui.ls_use_sensor.setChecked(self.lineUse)
        mymw.ui.ls_sensor_on.setChecked(self.lineUse)
        mymw.ui.ls_line_white.setChecked(self.lineWhite)
        mymw.ui.ls_left_side.setValue(self.edgeLeft)
        mymw.ui.ls_right_side.setValue(self.edgeRight)
        mymw.ui.ls_left_side_valid.setChecked(self.edgeLeftValid)
        mymw.ui.ls_right_side_valid.setChecked(self.edgeRightValid)
        ev = (self.edgeLeft + 2.5) * 20.0
        if (ev < 0):
          ev = 0
        elif ev >100:
          ev = 100
        mymw.ui.ls_left_bar.setValue(ev)
        ev = (self.edgeRight + 2.5) * 20.0
        if (ev < 0):
          ev = 0
        elif ev >100:
          ev = 100
        mymw.ui.ls_right_bar.setValue(ev)
        mymw.ui.ls_left_bar.setEnabled(self.edgeLeftValid)
        mymw.ui.ls_right_bar.setEnabled(self.edgeRightValid)
        mymw.ui.ls_follow_left.setChecked(self.followLeft)
        mymw.ui.ls_crossing_white.setValue(self.crossingWhiteCnt)
        mymw.ui.ls_crossing_black.setValue(self.crossingBlackCnt)
        mymw.ui.frame_ls_crossing_white.setEnabled(self.crossingWhite)
        mymw.ui.frame_ls_crossing_black.setEnabled(self.crossingBlack)
        mymw.ui.ls_power_high.setChecked(self.power_high)
        mymw.ui.ls_power_auto.setChecked(self.power_auto)
      # Show line sensor bar values
      mv = mymw.ui.line_disp_max_value.value() - 1
      if (self.dataReadLiv):
        self.dataReadLiv = False
        self.inUpdate = True
        if (self.lineValue[0] > mv):
          mymw.ui.line_bar_1.setValue(mv)
        elif self.lineValue[0] < 0:
          mymw.ui.line_bar_1.setValue(0)
        else:
          mymw.ui.line_bar_1.setValue(self.lineValue[0])
        if (self.lineValue[1] > mv):
          mymw.ui.line_bar_2.setValue(mv)
        elif self.lineValue[1] < 0:
          mymw.ui.line_bar_2.setValue(0)
        else:
          mymw.ui.line_bar_2.setValue(self.lineValue[1])
        if (self.lineValue[2] > mv):
          mymw.ui.line_bar_3.setValue(mv)
        elif self.lineValue[2] < 0:
          mymw.ui.line_bar_3.setValue(0)
        else:
          mymw.ui.line_bar_3.setValue(self.lineValue[2])
        if (self.lineValue[3] > mv):
          mymw.ui.line_bar_4.setValue(mv)
        elif self.lineValue[3] < 0:
          mymw.ui.line_bar_4.setValue(0)
        else:
          mymw.ui.line_bar_4.setValue(self.lineValue[3])
        if (self.lineValue[4] > mv):
          mymw.ui.line_bar_5.setValue(mv)
        elif self.lineValue[4] < 0:
          mymw.ui.line_bar_5.setValue(0)
        else:
          mymw.ui.line_bar_5.setValue(self.lineValue[4])
        if (self.lineValue[5] > mv):
          mymw.ui.line_bar_6.setValue(mv)
        elif self.lineValue[5] < 0:
          mymw.ui.line_bar_6.setValue(0)
        else:
          mymw.ui.line_bar_6.setValue(self.lineValue[5])
        if (self.lineValue[6] > mv):
          mymw.ui.line_bar_7.setValue(mv)
        elif self.lineValue[6] < 0:
          mymw.ui.line_bar_7.setValue(0)
        else:
          mymw.ui.line_bar_7.setValue(self.lineValue[6])
        if (self.lineValue[7] > mv):
          mymw.ui.line_bar_8.setValue(mv)
        elif self.lineValue[7] < 0:
          mymw.ui.line_bar_8.setValue(0)
        else:
          mymw.ui.line_bar_8.setValue(self.lineValue[7])
      # controller
      if (self.dataReadCtrl):
        # turning based on line-edge sensor
        self.dataReadCtrl = False
        self.inUpdate = True
        mymw.ui.ls_follow_left.setChecked(self.followLeft)
        #mymw.ui.reg_turn_use_2.setChecked(self.regActive)
        mymw.ui.reg_turn_KP_2.setValue(self.regKp)
        mymw.ui.reg_turn_tau_d_2.setValue(self.regTauD)
        mymw.ui.reg_turn_tau_i_2.setValue(self.regTauI)
        mymw.ui.reg_turn_alpha_2.setValue(self.regAlpha)
        mymw.ui.reg_turn_ilimit_2.setValue(self.regIMax)
        mymw.ui.reg_turn_out_limit_2.setValue(self.regOutMax)
        mymw.ui.reg_turn_step_on_2.setValue(self.regStepOn)
        mymw.ui.reg_turn_step_from_2.setValue(self.regStepFrom)
        mymw.ui.reg_turn_step_val_2.setValue(self.regStepTo)
        mymw.ui.reg_turn_step_vel_2.setValue(self.regStepVel)
        mymw.ui.reg_turn_LeadFwd_2.setChecked(self.regTurnLeadFwd)
    if (self.dataReadZ):
      self.dataReadZ = False
      if (self.regulZNumer[1] == 0.0):
        mymw.ui.reg_turn_numer_3.setText("%g" % (self.regulZNumer[0]))
        mymw.ui.reg_turn_denom_3.setText("1")
      else:
        mymw.ui.reg_turn_numer_3.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
        mymw.ui.reg_turn_denom_3.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
    if (self.dataReadZI):
      self.dataReadZI = False
      if (self.regulZINumer[1] == 0.0):
        mymw.ui.reg_turn_numer_4.setText("0")
        mymw.ui.reg_turn_denom_4.setText("1")
      else:
        mymw.ui.reg_turn_numer_4.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
        mymw.ui.reg_turn_denom_4.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    self.inUpdate = False
    # connected to robot
    mymw.ui.ls_line_white.setEnabled(mymw.ui.frame_batt_time.isEnabled())
    self.lock.release()
  #
  def helpbox(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Line sensor</span></p>
                <p>
                Values are difference between illuminated and not illuminated in A/D units.
                </p>
                <hr />
                <p>
                Left and right edge is in meters relative to center of robot.
                </p>
                ''');
      #about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
      self.about_box.setWindowTitle("Line sensor help")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()
  def max_value_changed(self):
    v = mymw.ui.line_disp_max_value.value()
    mymw.ui.line_bar_1.setMaximum(v)
    mymw.ui.line_bar_2.setMaximum(v)
    mymw.ui.line_bar_3.setMaximum(v)
    mymw.ui.line_bar_4.setMaximum(v)
    mymw.ui.line_bar_5.setMaximum(v)
    mymw.ui.line_bar_6.setMaximum(v)
    mymw.ui.line_bar_7.setMaximum(v)
    mymw.ui.line_bar_8.setMaximum(v)
    #print("line sensor maximum is " + str(mymw.ui.line_bar_8.maximum()))
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      #if ( mymw.ui.reg_bal_out_limit.value() < 0.5):
        #mymw.ui.label_out_limit.setStyleSheet("QLabel { background-color : red; }")
      #else:
        #mymw.ui.label_out_limit.setStyleSheet("QLabel { background-color : lightGray; }")
      #if (mymw.ui.reg_balvel_use.isChecked()):
        #mymw.ui.reg_bal_step_from.setValue(self.regStepVFrom)
        #mymw.ui.reg_bal_step_to.setValue(self.regStepVTo)
      #else:
        #mymw.ui.reg_bal_step_from.setValue(self.regStepFrom)
        #mymw.ui.reg_bal_step_to.setValue(self.regStepTo)
      self.lock.release()
  def regulatorUseClicked(self):
    self.configChanged()
    #mymw.ui.bal_regul_frame.repaint()
  def configChanged(self):
    # load the right value into step from-to widget
    self.inUpdate = True
    mymw.ui.ls_right_side.setEnabled(mymw.ui.ls_use_sensor.isChecked())
    mymw.ui.ls_left_side.setEnabled(mymw.ui.ls_use_sensor.isChecked())
    self.max_value_changed()
    self.inUpdate = False
    mymw.ui.ls_control_frame.repaint()
  def regulatorParamCancel(self):
    # cancel button pressed
    self.inEdit = False
  def applySettings(self):
    self.setWhiteLine()
    robot.conWrite("rl=%d %g %g %g %g %g %g %g %g %g %d %g %d\n" % (
        True, # mymw.ui.reg_turn_use_2.isChecked(), 
        mymw.ui.reg_turn_KP_2.value(), 
        mymw.ui.reg_turn_tau_i_2.value(), 
        mymw.ui.reg_turn_tau_d_2.value(), 
        mymw.ui.reg_turn_alpha_2.value(),
        mymw.ui.reg_turn_ilimit_2.value(),
        mymw.ui.reg_turn_step_on_2.value(),
        mymw.ui.reg_turn_step_from_2.value(),
        mymw.ui.reg_turn_step_val_2.value(),
        mymw.ui.reg_turn_step_vel_2.value(),
        mymw.ui.reg_turn_LeadFwd_2.isChecked(),
        mymw.ui.reg_turn_out_limit_2.value(),
        mymw.ui.ls_follow_left.isChecked()
        ))
    self.inEdit = False
    pass
  def sensorOnClicked(self):
    mymw.ui.ls_use_sensor.setChecked(mymw.ui.ls_sensor_on.isChecked())
    self.setWhiteLine()
  def setWhiteLine(self):
    # send white line assumption to robot
    robot.conWrite("lip=%d %d %d %d\n" % (
            mymw.ui.ls_use_sensor.isChecked(),
            mymw.ui.ls_line_white.isChecked(),
            mymw.ui.ls_power_high.isChecked(),
            mymw.ui.ls_power_auto.isChecked()
            ))
  def calibrateWhite(self):
    # send white line assumption to robot
    robot.conWrite("licw\n")
  def calibrateBlack(self):
    # send white line assumption to robot
    robot.conWrite("licb\n")
  def startLsMission(self):
    # set mission type and drive speed
    # set mission to 1 and start
    robot.conWrite("M=5\n")
    robot.conWrite("start\n")
    robot.mainStatus += " mission 5 started\n"
    robot.mainStatusSet = True
    pass
  def regulatorUseClicked(self):
    self.configChanged();
    mymw.turn_paint_space.repaint()
  def stepOrVelValueChanged(self):
    pass # save step and base vel in right config
    if (not self.inUpdate):
      if (mymw.ui.reg_vel_use.isChecked()):
        self.velBaseMotorReg = mymw.ui.reg_turn_step_vel.value()
      else:
        self.velBaseMotorV = mymw.ui.reg_turn_step_vel.value()
      # step size depend on turn and vel regulator use
      if (mymw.ui.reg_turn_use.isChecked()):
        self.stepValueTurnReg = mymw.ui.reg_turn_step_val.value()
      else:
        if (mymw.ui.reg_vel_use.isChecked()):
          self.stepValueMotorReg = mymw.ui.reg_turn_step_val.value()
        else:
          self.stepValueMotorV = mymw.ui.reg_turn_step_val.value()
    

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
#

class UIRDistance(object):
  # balance control
  #regActive = 0
  regKp = 0.0
  regTauD = 0.0
  regTauI = 0.0
  regAlpha = 0.0
  regIMax = 0.0
  regOutMax = 101
  regLeadFwd = True
  # sensor control
  sensorOn = False
  sensorUse1 = True
  #controlTurn = True
  #changeSign = False
  # step
  regStepTime = 0.0
  regStepFrom = 0.0
  regStepTo = 0.0
  regStepVel = 0.0 # velocity for wall follow (not valid for fwd sensor)
  #regStepPos = 0;
  # measurements
  distS1 = 0
  distS2 = 0
  irRaw = [0, 0]
  irCal20cm = [3000, 3000]
  irCal80cm = [480, 480]
  # data management
  dataRead = False
  dataDistRead = False
  inUpdate = False
  inEdit = False # control buttons
  inEdit2 = False # sensor buttons
  # Z-controllers
  regulZNumer = [1.0, 0.0]
  regulZDenom = [0.0, 0.0]
  regulZINumer = [1.0, 0.0]
  regulZIDenom = [0.0, 0.0]
  dataReadZ = False
  dataReadZI = False
  # help
  about_box = None
  # resource lock
  lock = threading.RLock()
  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      #snprintf(s, MSL, "rgd 1 %g %g %g %g %g %g %g %g %g %d %g %d %d\n", // use kp ti td al limit step_on step_off step_val step_vel
           #regul_wall_kp, regul_wall_ti, regul_wall_td, regul_wall_alpha, regul_wall_i_limit,
           #regul_wall_step_time, regul_wall_step_from, regul_wall_step_to, regul_wall_step_vel, 
           #regul_wall_LeadFwd, regul_wall_u_limit, regul_wall_use, regul_wall_sensor1

      if gg[0] == "rgd":
        #self.regActive = int(gg[1],0)
        self.dataRead = True
        self.regKp = float(gg[2])
        self.regTauI = float(gg[3])
        self.regTauD = float(gg[4])
        self.regAlpha = float(gg[5])
        self.regIMax = float(gg[6])
        self.regStepTime = float(gg[7])
        self.regStepFrom = float(gg[8])
        self.regStepTo = float(gg[9])
        self.regStepVel = float(gg[10])
        self.regLeadFwd = int(gg[11], 0)
        self.regOutMax = float(gg[12])
        self.sensorOn = int(gg[13], 0)
        self.sensorUse1 = int(gg[14],0)
      elif gg[0] == "irc":
        self.distS1 = float(gg[1])
        self.distS2 = float(gg[2])
        self.irRaw[0] = int(gg[3],0)
        self.irRaw[1] = int(gg[4],0)
        self.irCal20cm[0] = int(gg[5],0)
        self.irCal80cm[0] = int(gg[6],0)
        self.irCal20cm[1] = int(gg[7],0)
        self.irCal80cm[1] = int(gg[8],0)
        self.dataDistRead = True
      elif gg[0] == "rgdz":
        self.regulZDenom[1] = float(gg[1])
        self.regulZNumer[0] = float(gg[2])
        self.regulZNumer[1] = float(gg[3])
        self.dataReadZ = True
      elif gg[0] == "rgdzi":
        self.regulZIDenom[1] = float(gg[1])
        self.regulZINumer[0] = float(gg[2])
        self.regulZINumer[1] = float(gg[3])
        self.dataReadZI = True 
      else:
        used = False
    except:
      print("IR sensor: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def showData(self):
    self.lock.acquire()
    if (self.dataDistRead):
      self.dataDistRead = False
      self.inUpdate = True
      mymw.ui.ir_d1_meters.setText(str(self.distS1))
      mymw.ui.ir_d2_meters.setText(str(self.distS2))
      mymw.ui.ir_bar_1.setValue(self.irRaw[0])
      mymw.ui.ir_bar_2.setValue(self.irRaw[1])
      mymw.ui.ir_d1_raw.setValue(self.irRaw[0])
      mymw.ui.ir_d2_raw.setValue(self.irRaw[1])
      if not self.inEdit2:
        mymw.ui.ir_d1_20cm.setValue(self.irCal20cm[0])
        mymw.ui.ir_d2_20cm.setValue(self.irCal20cm[1])
        mymw.ui.ir_d1_80cm.setValue(self.irCal80cm[0])
        mymw.ui.ir_d2_80cm.setValue(self.irCal80cm[1])
      #print("distance is " + str(self.distS1) + ", " + str(self.distS2))
      self.inUpdate = False
    if self.dataRead:
      self.dataRead = False
      if (not self.inEdit):
        mymw.ui.reg_wall_KP.setValue(self.regKp)
        mymw.ui.reg_wall_tau_d.setValue(self.regTauD)
        mymw.ui.reg_wall_tau_i.setValue(self.regTauI)
        mymw.ui.reg_wall_alpha.setValue(self.regAlpha)
        mymw.ui.reg_wall_ilimit.setValue(self.regIMax)
        mymw.ui.reg_wall_out_limit.setValue(self.regOutMax)
        mymw.ui.reg_wall_step_time.setValue(self.regStepTime)
        mymw.ui.reg_wall_step_from.setValue(self.regStepFrom)
        mymw.ui.reg_wall_step_to.setValue(self.regStepTo)
        mymw.ui.reg_wall_LeadFwd.setChecked(self.regLeadFwd)
        mymw.ui.wall_sensor_on.setChecked(self.sensorOn)
        mymw.ui.wall_follow_left.setChecked(self.sensorUse1)
        #mymw.ui.wall_control_sign.setChecked(self.changeSign)
    if (False and self.dataReadZ):
      self.dataReadZ = False
      if (self.regulZNumer[1] == 0.0):
        mymw.ui.reg_wall_numer.setText("%g" % (self.regulZNumer[0]))
        mymw.ui.reg_wall_denom.setText("1")
      else:
        mymw.ui.reg_wall_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
        mymw.ui.reg_wall_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
    if (False and self.dataReadZI):
      self.dataReadZI = False
      if (self.regulZINumer[1] == 0.0):
        mymw.ui.reg_wall_numer_2.setText("0")
        mymw.ui.reg_wall_denom_2.setText("1")
      else:
        mymw.ui.reg_wall_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
        mymw.ui.reg_wall_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))        
    # enable buttons
    if (not mymw.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      mymw.ui.wall_apply.setEnabled(False)
      mymw.ui.wall_start.setEnabled(False)
      mymw.ui.wall_cancel.setEnabled(False)
      mymw.ui.wall_edit.setEnabled(True)
      mymw.ui.ir_apply.setEnabled(False)
      mymw.ui.ir_cancel.setEnabled(False)
      mymw.ui.ir_edit.setEnabled(True)
    else:
      mymw.ui.wall_apply.setEnabled(self.inEdit)
      mymw.ui.wall_cancel.setEnabled(self.inEdit)
      mymw.ui.wall_edit.setEnabled(not self.inEdit)
      mymw.ui.wall_start.setEnabled(not self.inEdit)
      mymw.ui.ir_apply.setEnabled(self.inEdit2)
      mymw.ui.ir_cancel.setEnabled(self.inEdit2)
      mymw.ui.ir_edit.setEnabled(not self.inEdit2)
    self.lock.release()
  def dataChangedManually(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      self.lock.release()
  def dataEditCal(self):
    if (not robot.timerUpdate):
      self.lock.acquire()
      self.inEdit2 = True
      self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def regulatorUseClicked(self):
    self.configChanged()
    #mymw.ui.bal_regul_frame.repaint()
  def configChanged(self):
    # load the right value into step from-to widget
    #self.inUpdate = True
    #self.inUpdate = False
    mymw.ui.wall_control_frame.repaint()
  def paramCancel(self):
    # cancel button pressed
    self.inEdit = False
  def paramCancelCal(self):
    # cancel button pressed
    self.inEdit2 = False
  def helpbox(self):
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Distance sensor</span></p>
                <p>
                May in the future be used for wall following
                </p>''');
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()
  def helpboxFollowWall(self):
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(mymw)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Follow wall control</span></p>
                <p>
                The follow wall control parameters are active only from this page or in a mission line with a wall option </p>
                <p>
                The usual PID control parameters </p>
                <hr />
                <p>
                The sensor can be either of the (optional) two IR distance sensors.
                </p>''');
      self.about_box.setWindowTitle("regbot motor velocity")
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
    self.about_box.show()
  def start(self):
    # set mission to 1 and start
    robot.conWrite("M=6\n")
    robot.conWrite("start\n")
    robot.mainStatus += " mission 6 (follow wall) started\n"
    robot.mainStatusSet = True
  def paramApply(self):
    robot.conWrite("rd=%d %g %g %g %g %g %g %g %g %g %d %g %d %d\n" % (
        True, #mymw.ui.reg_wall_use.isChecked(), 
        mymw.ui.reg_wall_KP.value(), 
        mymw.ui.reg_wall_tau_i.value(), 
        mymw.ui.reg_wall_tau_d.value(), 
        mymw.ui.reg_wall_alpha.value(),
        mymw.ui.reg_wall_ilimit.value(),
        mymw.ui.reg_wall_step_time.value(),
        mymw.ui.reg_wall_step_from.value(),
        mymw.ui.reg_wall_step_to.value(),
        mymw.ui.reg_wall_step_vel.value(),
        mymw.ui.reg_wall_LeadFwd.isChecked(),
        mymw.ui.reg_wall_out_limit.value(),
        mymw.ui.wall_sensor_on.isChecked(),
        mymw.ui.wall_follow_left.isChecked()
        ))
    #mymw.ui.reg_turn_apply.setEnabled(False)
    #self.regTurn.stepOrVelValueChanged()
    self.inEdit = False
  def paramApplyCal(self):
    robot.conWrite("irc=%g %g %g %g\n" % (
        mymw.ui.ir_d1_20cm.value(), 
        mymw.ui.ir_d1_80cm.value(), 
        mymw.ui.ir_d2_20cm.value(), 
        mymw.ui.ir_d2_80cm.value()
        ))
    #mymw.ui.reg_turn_apply.setEnabled(False)
    #self.regTurn.stepOrVelValueChanged()
    self.inEdit2 = False

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////    


class URobot(object):
  con = serial.Serial()
  wificlient = socket.socket()
  # status data
  time = 0
  msgCnt = 0;
  msgCntLast = 0;
  timeLast = 0;
  timeLastCnt = 0;
  #
  mainSetReset = False
  #
  statusRq = 0 # set status from robot
  #
  mainStatus = ""
  mainStatusSet = False
  dataRxCnt = 0
  dataTxCnt = 0
  #
  mission = UMission()
  log = ULog()
  imu = UImu()
  drive = UDrive() 
  regTurn = URegTurn()
  regVel = URegVel()
  regPosition = URegPosition()
  regBal = URegBal()
  regSSBal = URegSSBal()
  lineSensor = ULineSensor()
  irDist = UIRDistance();
  info = UInfo()
  timerUpdate = False
  failCnt = 0
  wifiPort = 24001
  wifiFailCnt = 0
  wifiConnected = False
  decodeLock = threading.RLock()
  statusUpdateTime = 0
  statusUpdateIdx = 0;
  #
  def __init__(self):
    self.stop = threading.Event()
    self.doRead = threading.Thread(target=self.readThread, name="regbot_usb_reader")
    self.doRead.start()
    self.doReadWiFi = threading.Thread(target=self.readThreadWiFi, name="regbot_wifi_reader")
    self.doReadWiFi.start()
  def terminate(self):
    self.stop.set()
    self.close()
    self.doRead.join(2)
    self.wificlient.close()
    self.doReadWiFi.join(2)
  def usbopen(self, name):
    if (not self.con.isOpen()):
      self.con.port = str(name)
      self.con.timeout = 0.5
      self.conWriteTimeout = 0.5
      #print("Trying to open:" + self.con.port)
      try:
        self.con.open()
        print("usb - opened OK.")
        self.mainStatus += "USB Connected OK\n"
        self.mainStatusSet = True
        self.failCnt = 0
      except:
        if self.failCnt < 5:
          print("usb failed to open usb-port :" + str(name))
        self.failCnt += 1
        #mymw.ui.connect_usb.setChecked(False)
        self.mainStatus += "usb Failed to connect to " + str(name) + "\n"
        self.mainStatusSet = True
    if self.con.isOpen():
      #if not mymw.ui.connect_usb.isChecked():
      #  mymw.ui.connect_usb.setChecked(True)
      self.con.flushInput()
      self.con.flushOutput()
      #flags = fcntl(self.con, F_GETFL) # get current p.stdout flags
      #fcntl(self.con, F_SETFL, flags & ~O_NONBLOCK)
      self.requestData()
      #self.getStatus()
    pass
  
  def close(self):
    if self.con.isOpen():
      print("stopping push S=0")
      self.conWrite("S=0\n")
      self.con.close()
      mymw.ui.statusbar.showMessage("Robot client - disconnected")
      self.mainStatus += "Robot is disconnected\n"
      self.mainStatusSet = True
    pass
  
  def wifiOpen(self):
    if not self.wifiConnected:
      hopo = mymw.ui.wifi_host_name.text().split(':')
      try:
        self.wifiPort = int(str(hopo[1]), 0)
      except:
        self.wifiPort = 24001
        print("wifi no ':' or port number found in '" + hopo[0] + "'")
      try:
        #hopo = mymw.ui.wifi_host_name.text().split(':')
        self.wificlient.connect((hopo[0], self.wifiPort))
        print("wifi connect did not fail")
        self.wifiFailCnt = 0
        self.wifiConnected = True
      except:
        if (self.wifiFailCnt < 3):
          print("wifi Failed to connect to " + hopo[0] + ", port " + str(self.wifiPort))
        self.wifiFailCnt += 1
        pass
    if self.wifiConnected:
      print("wifi is open")
      mymw.ui.statusbar.showMessage("wifi client - connected")
      self.mainStatus += "wifi is connected\n"
      self.mainStatusSet = True
      pass
    pass
  
  def wifiClose(self):
    if (self.wifiConnected):
      print("wifi stopping")
      self.wifiWrite("S=0\n")
      self.wifiConnected = False
      self.wificlient.close()
      mymw.ui.statusbar.showMessage("wifi client - disconnected")
      self.mainStatus += "wifi is disconnected\n"
      self.mainStatusSet = True
    pass

  def connect_usb_changed(self):
    if mymw.ui.connect_usb.isChecked():
      #print("trying to open connection to regbot") 
      if (not self.isConnected()):
        name = mymw.ui.usb_device_name.text()
        self.usbopen(name)
        if self.isConnected():
          #mymw.ui.usb_port_label.setText('USB port (open)')
          mymw.ui.statusbar.showMessage("Robot client - connected")
        else:
          #mymw.ui.usb_port_label.setText('USB port (connect failed)')
          mymw.ui.statusbar.showMessage("Robot client starting - not connected")
          #self.ui.connect_usb.setCheckState(QtCore.Qt.Unchecked)
      pass
    else:
      if (self.isConnected()):
        print("usb closing connection to regbot") 
        self.close()
        #mymw.ui.usb_port_label.setText('USB port (closed)')
    pass

  def connect_wifi_changed(self):
    if mymw.ui.connect_wifi.isChecked():
      #print("trying to open connection to regbot") 
      if not self.wifiConnected:
        self.wifiOpen()
        if self.wifiConnected:
          #mymw.ui.usb_port_label.setText('wifi port (open)')
          mymw.ui.statusbar.showMessage("wifi client - connected")
        else:
          #mymw.ui.usb_port_label.setText('wifi port (connect failed)')
          mymw.ui.statusbar.showMessage("wifi client starting - not connected")
          #self.ui.connect_usb.setCheckState(QtCore.Qt.Unchecked)
      pass
    else:
      if self.wifiConnected:
        print("wifi closing connection to regbot") 
        self.close()
        #mymw.ui.usb_port_label.setText('USB port (closed)')
    pass
  
  def isConnected(self):
    return self.con.isOpen()
  # use this in place of con.write
  def conWrite(self, s):
    isSend = False
    if (self.con.isOpen()):
      self.usbWrite(s)
      isSend = self.con.isOpen()
    elif self.wifiConnected:
      # debug
      print("# sending to wifi " + s)
      # debug end
      self.wifiWrite(s)
      isSend = self.wifiConnected
    if (isSend):
      self.dataTxCnt += 1 #len(s)
      if (mymw.ui.main_show_all_tx.isChecked()):
        #print("- added: " + s)
        self.mainStatus += "\r\n" + str(s)
        self.mainStatusSet = True
    else:
      self.mainStatus += "not connected, could not send " + s
      self.mainStatusSet = True
    pass
  #
  ### send string to USB connection
  def usbWrite(self, s):
    if self.con.isOpen():
      n = len(s)
      #print("# sending " + s)
      if (n > 0):
        try:
          n = self.con.write(s)
          if (n == 0):
            raise Exception("Write error")
        except:
          self.con.close()
          print("URobot conWrite - closed connection")
          mymw.ui.statusbar.showMessage("Robot client - not connected")
    pass
  #
  ### send string to wifi socket
  def wifiWrite(self, s):
    n = len(s)
    #print("# sending " + s)
    if (n > 0):
      m = 0;
      try:
        while m < n:
          d = self.wificlient.send(s[m:])
          if (d == 0):
            raise Exception("Write error")
          m += d
      except:
        self.wificlient.close()
        print("wifi - closed connection")
        mymw.ui.statusbar.showMessage("wifi connection broken")
    #print("# send " + s)
    pass
  #
  ### request data from robot
  def requestData(self):
    #print("trying to send S=" + str(mymw.ui.main_push_interval.value()))
    self.conWrite("i=0\r\n") # turn off interactive mode
    self.conWrite("u0\r\n") # request software version
    self.conWrite("u4\r\n") # request static robot info
    # request regular status updates
    self.conWrite("S=" + str(mymw.ui.main_push_interval.value()) + "\r\n")
  #
  ### interpret data from robot
  def decodeCommand(self, got, n):
    if n > 0:
      #print(got)
      self.dataRxCnt += n
      if (mymw.ui.main_show_all_rx.isChecked()):
        self.mainStatus += got
        self.mainStatusSet = True
    #if (got[0] == '<'):
      #print("fik " + str(n) + " bytes: " + got)
    if n > 3:
      self.msgCnt += 1
      gg = got.split()
      if (self.mission.readUserMissionLine(got)):
        # read user defined mission lines
        # print("got mission line - now " + str(len(self.mission.mLines)) + " lines in mLines")
        pass
      elif self.imu.readData(gg):
        pass
      elif gg[0] == "hbt":
        self.time = float(gg[1])
      elif self.drive.readData(gg):
        pass
      elif self.regTurn.readData(gg):
        pass
      elif self.regVel.readData(gg):
        pass
      elif self.mission.readData(gg, got):
        pass
      elif self.info.readData(gg):
        pass
      elif self.log.readData(gg, got):
        pass
      elif self.regBal.readData(gg):
        pass
      elif self.regSSBal.readData(gg):
        pass
      elif self.lineSensor.readData(gg):
        pass
      elif self.irDist.readData(gg):
        pass
      elif self.regPosition.readData(gg):
        pass
      else:
        if (not mymw.ui.main_show_all_rx.isChecked()):
          self.mainStatus += got
          self.mainStatusSet = True
    pass


  def readThread(self):
    count = 0
    m = 0
    n = 0
    c = '\0'
    self.threadRunning = True
    print("thread running")
    got = ""
    while (not self.stop.is_set()):
      if self.con.isOpen():
        m = m + 1
        n = 0
        if (c == '\n'):
          got = ""
        c = '\0'
        try:
          while (c != '\n'):
            c = self.con.read(1)
            if (c >= ' ' or c == '\n'):
              got = got + c
          #print("got (" + str(m) + ")=" + got)
          n = len(got)
        except:
          ##print("read " + str(m) + " returned 0 bytes - " + str(n) + "- closing")
          m = m + 1
          sleep(0.01)
        self.decodeCommand(got, n)
      else:
        time.sleep(0.1)
    print("read thread ended")
    self.threadRunning = False
    
  def readThreadWiFi(self):
    count = 0
    m = 0
    n = 0
    c = '\0'
    self.threadRunningWiFi = True
    print("thread wifi running")
    got = ""
    self.wificlient.settimeout(0.5)
    while (not self.stop.is_set()):
      if self.wifiConnected:
        m = m + 1
        n = 0
        if (c == '\n'):
          got = ""
        c = '\0'
        try:
          while (c != '\n'):
            c = self.con.recv(1)
            if (len(c) > 0):
              if (c >= ' ' or c == '\n'):
                # filter all control characters but newline
                got = got + c
          #print("got (" + str(m) + ")=" + got)
          n = len(got)
        except:
          ##print("read " + str(m) + " returned 0 bytes - " + str(n) + "- closing")
          m = m + 1
          sleep(0.01)
        self.decodeCommand(got, n)
      else:
        time.sleep(0.1)
    print("read thread ended")
    self.threadRunning = False
    pass
  #
  ### 
  #def mainSettingRead(self):
    #self.mainSetReset = True
  #
  # start a mission
  def mainSettingStart(self):
    self.conWrite("start\r\n")
  #
  ### stop a mission
  def mainSettingStop(self):
    self.conWrite("stop\r\n")
  #
  def regulatorParamTurnApply(self):
    self.conWrite("rt=%d %g %g %g %g %g %g %g %g %g %d %g\n" % (
        mymw.ui.reg_turn_use.isChecked(), 
        mymw.ui.reg_turn_KP.value(), 
        mymw.ui.reg_turn_tau_i.value(), 
        mymw.ui.reg_turn_tau_d.value(), 
        mymw.ui.reg_turn_alpha.value(),
        mymw.ui.reg_turn_ilimit.value(),
        mymw.ui.reg_turn_step_on.value(),
        mymw.ui.reg_turn_step_off.value(),
        mymw.ui.reg_turn_step_val.value(),
        mymw.ui.reg_turn_step_vel.value(),
        mymw.ui.reg_turn_LeadFwd.isChecked(),
        mymw.ui.reg_turn_out_limit.value()
        ))
    #mymw.ui.reg_turn_apply.setEnabled(False)
    #self.regTurn.stepOrVelValueChanged()
    self.regTurn.inEdit = False
    
  def regulatorParamVelApply(self):
    self.conWrite("rv=%d %g %g %g %g %g %g %g %g %g %d %d %g\n" % (
      mymw.ui.reg_vel_use.isChecked(), 
      mymw.ui.reg_vel_KP.value(), 
      mymw.ui.reg_vel_tau_i.value(), 
      mymw.ui.reg_vel_tau_d.value(), 
      mymw.ui.reg_vel_alpha.value(),
      mymw.ui.reg_vel_integrate_max.value(),
      mymw.ui.reg_vel_steptime.value(), 
      mymw.ui.reg_vel_step_from.value(), 
      mymw.ui.reg_vel_step_to.value(),
      mymw.ui.reg_vel_acc_limit.value(), 
      mymw.ui.reg_vel_est_use.isChecked(),
      mymw.ui.reg_vel_LeadFwd.isChecked(),
      mymw.ui.reg_vel_volt_limit.value()
      ))
    self.regVel.inEdit = False
  def regulatorParamSSApply(self):
    self.conWrite("rs=%d %g %g %g %g %g %g %g %g %g %d\n" % (
      mymw.ui.reg_bal_ss_use.isChecked(), 
      mymw.ui.reg_bal_ss_k_tilt.value(), 
      mymw.ui.reg_bal_ss_k_gyro.value(), 
      mymw.ui.reg_bal_ss_k_pos.value(), 
      mymw.ui.reg_bal_ss_k_vel.value(),
      mymw.ui.reg_bal_ss_k_motor.value(),
      mymw.ui.reg_bal_ss_out_limit.value(), 
      mymw.ui.reg_bal_ss_steptime.value(), 
      mymw.ui.reg_bal_ss_step_from.value(), 
      mymw.ui.reg_bal_ss_step_to.value(),
      mymw.ui.reg_bal_ss_step.value(),
      ))
    self.regSSBal.inEdit = False
  def regulatorParamBalApply(self):
    self.conWrite("rb=%d %g %g %g %g %g %g %g %g %d %g %d\n" % (
      mymw.ui.reg_bal_use.isChecked(), 
      mymw.ui.reg_bal_KP.value(), 
      mymw.ui.reg_bal_tau_i.value(), 
      mymw.ui.reg_bal_tau_d.value(), 
      mymw.ui.reg_bal_alpha.value(),
      mymw.ui.reg_bal_integrate_max.value(),
      mymw.ui.reg_bal_steptime.value(), 
      mymw.ui.reg_bal_step_from.value(), 
      mymw.ui.reg_bal_step_to.value(),
      mymw.ui.reg_bal_LeadFwd.isChecked(),
      mymw.ui.reg_bal_out_limit.value(),
      mymw.ui.reg_bal_LeadGyro.isChecked()
      ))
    # balance mission velocity
    self.conWrite("rm=%d %g %g %g %g %g %g %d %g\n" % (
      mymw.ui.reg_balvel_use.isChecked(),
      mymw.ui.reg_mvel_KP.value(), 
      mymw.ui.reg_mvel_tau_i.value(), 
      mymw.ui.reg_mvel_tau_d.value(), 
      mymw.ui.reg_mvel_alpha.value(),
      mymw.ui.reg_mvel_zeta.value(),
      mymw.ui.reg_mvel_integrate_max.value(),
      mymw.ui.reg_mvel_LeadFwd.isChecked(),
      mymw.ui.reg_mvel_out_limit.value()
      ))
    self.regBal.inEdit = False
  def robotIdApply(self):
    # print("robotIDapply clicked")
    self.conWrite("rid=%d %g %g %d %g %g %g %d %g\n" % (
      mymw.ui.robot_id.value(),
      mymw.ui.robot_base.value(),
      mymw.ui.robot_gear.value(),
      mymw.ui.robot_pulse_per_rev.value(),
      mymw.ui.robot_wheel_radius_left.value(),
      mymw.ui.robot_wheel_radius_right.value(),
      mymw.ui.robot_balance_offset.value(),
      mymw.ui.robot_on_battery.isChecked(),
      mymw.ui.robot_battery_idle_volt.value()
      ))
    self.info.inEdit = False
    #mymw.ui.save_id_on_robot.setEnabled(False)
  def doGyroOffset(self):
    mymw.ui.imu_gyro_offset_done.setChecked(False)
    self.conWrite("gyroo\n")
    
  def mainStatusClear(self):
    self.mainStatus = ""
    self.mainStatusSet = True;
    self.dataRxCnt = 0
    self.dataTxCnt = 0
  def mainSettingHelp(self):
    print("help clicked")
    self.conWrite("h\n")    
  def requestStatusUpdate(self):
    dt = timeit.timeit() - self.statusUpdateTime
    if (dt > 0.5):
      self.statusUpdateTime = timeit.timeit()
      wi = mymw.ui.tabPages.currentWidget()
      # mission
      if wi == self.ui.tabPages.indexOf(self.ui.tab_6):
        pass
      elif wi ==  self.ui.tabPages.indexOf(self.ui.tab_13): # follow wall
        self.conWrite("x3\n")
        pass
      elif wi ==  self.ui.tabPages.indexOf(self.ui.tab_9): # follow edge
        self.conWrite("x3\n")
        pass
      elif wi ==  self.ui.tabPages.indexOf(self.ui.tab_8): # SS
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_5): # balance
        if self.statusUpdateIdx > 1:
          self.statusUpdateIdx = 0
        if (self.statusUpdateIdx == 0):
          self.conWrite("x10\n")
        if (self.statusUpdateIdx == 1):
          self.conWrite("x12\n")
        self.statusUpdateIdx += 1
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_4): # turn
        self.conWrite("x1\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.speed_ctrl): # velocity
        self.conWrite("x12\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_12): # position
        self.conWrite("x5\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_11): # IR-distance
        self.conWrite("x3\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_10): # edge
        self.conWrite("x2\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_3): # IMU
        self.conWrite("u1\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_2): # robot
        if self.statusUpdateIdx > 2:
          self.statusUpdateIdx = 0
        if (self.statusUpdateIdx == 0):
          self.conWrite("u1\n") # robot sensor like encoder and pose
        if (self.statusUpdateIdx == 1):
          self.conWrite("u4\n") # robot gear etc
        if (self.statusUpdateIdx == 2):
          self.conWrite("u9\n") # motor current and motor voltage
        self.statusUpdateIdx += 1
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab): # log
        self.conWrite("u3\n")
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_7): # Z
        if self.statusUpdateIdx > 4:
          self.statusUpdateIdx = 0
        if (self.statusUpdateIdx == 0):
          self.conWrite("x6\n")
        elif (self.statusUpdateIdx == 1):
          self.conWrite("x7\n")
        elif (self.statusUpdateIdx == 2):
          self.conWrite("x8\n")
        elif (self.statusUpdateIdx == 3):
          self.conWrite("x9\n")
        elif (self.statusUpdateIdx == 4):
          self.conWrite("x13\n")
        self.statusUpdateIdx += 1
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.tab_14): # wifi
        pass
      elif wi == self.ui.tabPages.indexOf(self.ui.main): # debug
        pass
    pass
  ## timer update to update display
  def timerUpdate(self):
    self.timerUpdate = True
    self.regTurn.showData()
    self.mission.showData()
    self.log.showData()
    self.imu.showData()
    self.drive.showData()
    self.regPosition.showData()
    self.regTurn.showData()
    self.regVel.showData()
    self.regBal.showData()
    self.regSSBal.showData()
    self.lineSensor.showData()
    self.irDist.showData()
    self.info.showData()
    self.requestStatusUpdate()
    if (self.mainStatusSet):
      self.mainStatusSet = False
      if (len(self.mainStatus) > 32000):
        self.mainStatus = "# status truncated\n"
      mymw.ui.main_status.setPlainText(self.mainStatus)
      mymw.ui.main_status.verticalScrollBar().setValue(mymw.ui.main_status.verticalScrollBar().maximum())
      mymw.ui.Main_status_log.setPlainText(self.mainStatus)
      mymw.ui.Main_status_log.verticalScrollBar().setValue(mymw.ui.Main_status_log.verticalScrollBar().maximum())
      #mymw.ui.main_status.ensureCursorVisible()
    mymw.ui.main_rxCnt.setValue(self.dataRxCnt)
    mymw.ui.main_txCnt.setValue(self.dataTxCnt)
    if (mymw.ui.connect_usb.isChecked() and not self.con.isOpen()):
      #mymw.ui.connect_usb.setChecked(False)
      mymw.ui.statusbar.showMessage("Connection closed - e.g. no hart beat")
      mymw.ui.usb_port_label.setText('USB port')
    if self.mission.missionManually and self.con.isOpen():
      self.conWrite("M=" + str(mymw.ui.main_mission_2.value()) + "\n")
      self.mission.missionManually = False
    mymw.ui.main_time.setValue(self.time)
    self.timerUpdate = False
    if (self.time != self.timeLast or self.msgCnt != self.msgCntLast):
      # is the connection alive - based on hartbeat or other messages
      self.timeLastCnt = 0
      self.msgCntLast = self.msgCnt
      self.timeLast = self.time
      if (not mymw.ui.frame_batt_time.isEnabled()):
        mymw.ui.frame_batt_time.setEnabled(True)
        print('Got active communication with robot ' + mymw.ui.robot_id_main.text())
        self.mainStatus += 'Robot ' + mymw.ui.robot_id_main.text() + " active\n"
        self.mainStatusSet = True;
      mymw.ui.frame_usb_connect.repaint() # setEnabled(self.isConnected())
      mymw.ui.frame_wifi_connect.repaint() # Enabled(self.wifiConnected and not self.isConnected())
    elif (self.con.isOpen() or  self.wifiConnected):
      self.timeLastCnt = self.timeLastCnt + 1
      #print("--    connection count=" + str(self.timeLastCnt))
      if (self.timeLastCnt >= 30):
        if (mymw.ui.frame_batt_time.isEnabled()):
          mymw.ui.frame_batt_time.setEnabled(False)
          print('No  active communication with robot ' + mymw.ui.robot_id_main.text())
          self.mainStatus += 'Robot ' + mymw.ui.robot_id_main.text() + " silent\n"
          self.mainStatusSet = True
        if (self.timeLastCnt >= 40):
          self.con.close()
          self.timeLastCnt = 0;
      mymw.ui.frame_usb_connect.repaint() # setEnabled(self.isConnected())
      mymw.ui.frame_wifi_connect.repaint() # Enabled(self.wifiConnected and not self.isConnected())
      pass
    else:
      # no connection, so no timeout
      self.timeLastCnt = self.timeLastCnt + 1
      #print("-- no connection count=" + str(self.timeLastCnt))
      if mymw.ui.connect_usb.isChecked():
        # try autoconnect
        if self.timeLastCnt >= 100:
          # try if open works now
          robot.usbopen(mymw.ui.usb_device_name.text())
          self.timeLastCnt = 0
      # make sure to show
      if (mymw.ui.frame_batt_time.isEnabled()):
        mymw.ui.frame_batt_time.setEnabled(False)
    self.timeLast = self.time
    pass
        

  def sendCmd(self):
    s = mymw.ui.main_send_txt.text() + "\n"
    self.conWrite(s.toAscii())
    #print("Urobot: send " + s + str(len(s)))
  def mainMissionChanged2(self):
    self.missionChanged(mymw.ui.main_mission_2.value())
  def missionChanged(self, m):
    if (self.con.isOpen()):
      self.conWrite("M=" + str(m) + "\n")
  def mainPushInterval(self):
    self.conWrite("S=" + str(mymw.ui.main_push_interval.value()) + "\n")
  def robotPoseReset(self):
    self.conWrite("posec\n")
    self.drive.poseReset()
  def logGet(self):
    self.conWrite("log get\n")
  def setLogFlag_lac(self):
    self.setLogFlag("acc", mymw.ui.log_lac.isChecked())
  def setLogFlag_lbt(self):
    self.setLogFlag("bat", mymw.ui.log_lbt.isChecked())
  def setLogFlag_line(self):
    self.setLogFlag("line", mymw.ui.log_line.isChecked())
  def setLogFlag_dist(self):
    self.setLogFlag("dist", mymw.ui.log_distance.isChecked())
  def setLogFlag_lct(self):
    self.setLogFlag("ct", mymw.ui.log_lct.isChecked())
  def setLogFlag_lgy(self):
    self.setLogFlag("gyro", mymw.ui.log_lgy.isChecked())
  def setLogFlag_lma(self):
    self.setLogFlag("motA", mymw.ui.log_lma.isChecked())
  def setLogFlag_lme(self):
    self.setLogFlag("enc", mymw.ui.log_lme.isChecked())
  def setLogFlag_lmr(self):
    self.setLogFlag("mvel", mymw.ui.log_lmr.isChecked())
  def setLogFlag_lms(self):
    self.setLogFlag("mis", mymw.ui.log_lms.isChecked())
  def setLogFlag_lmv(self):
    self.setLogFlag("motV", mymw.ui.log_lmv.isChecked())
  def setLogFlag_lvr(self):
    self.setLogFlag("motR", mymw.ui.log_lvr.isChecked())
  def setLogFlag_ltr(self):
    self.setLogFlag("tr", mymw.ui.log_turn_rate.isChecked())
  def setLogFlag_lex(self):
    self.setLogFlag("extra", mymw.ui.log_lex.isChecked())
  def setLogFlag_lbc(self):
    self.setLogFlag("bcl", mymw.ui.log_lbc.isChecked())
  def setLogFlag_lpo(self):
    self.setLogFlag("pose", mymw.ui.log_lpo.isChecked())
  def setLogFlag(self, flag, checked):
    if (not self.timerUpdate):
      c = '-'
      if (checked):
        c = '+'
      self.conWrite("log" + c + flag + "\n")
  def setLogInterval(self):
    if (not self.timerUpdate):
      self.conWrite("s=" + str(mymw.ui.log_interval.value()) + " " + str(int(mymw.ui.log_allow.isChecked())) + "\n")
  def statusWhileRunning(self):
    if (not self.timerUpdate):
      if (mymw.ui.main_status_while_running.isChecked()):
        self.conWrite("R=1\n")
      else:
        self.conWrite("R=0\n")
  def setLogFileName(self):
    filename = QtGui.QFileDialog.getOpenFileName(mymw,'logfile to use', '', 'log (*.txt)')
    if (filename is not None and len(filename) > 0):
      print("saved log to '" + filename + "'")
      mymw.ui.log_filename.setText(filename)
  def logSave(self):
    if (mymw.ui.log_save_config.isChecked()):
      self.configurationFileSave('.regbotConfigTemp.ini', False)
    self.log.logSave()
  def mainStatusStop(self):
    mymw.ui.main_push_interval.setValue(0)
  def mainStatusStart(self):
    mymw.ui.main_push_interval.setValue(50)
  def regVelStart(self):
    # set mission to 1 and start
    self.conWrite("M=0\n")
    self.conWrite("start\n")
    self.mainStatus += " mission 0 started\n"
    self.mainStatusSet = True
  def regTurnStart(self):
    # set mission to 1 and start
    self.conWrite("M=1\n")
    self.conWrite("start\n")
    self.mainStatus += " mission 1 started\n"
    self.mainStatusSet = True
  def regBalStart(self):
    # set mission to 1 and start
    self.conWrite("M=2\n")
    self.conWrite("start\n")
    self.mainStatus += " mission 2 started\n"
    self.mainStatusSet = True
  def regSSBalStart(self):
    # set mission to 1 and start
    self.conWrite("M=4\n")
    self.conWrite("start\n")
    self.mainStatus += " mission 4 started\n"
    self.mainStatusSet = True
  def regMissionStart(self):
    # set mission to 1 and start
    self.conWrite("M=3\n")
    self.conWrite("start\n")
    self.mainStatus += " mission 3 started\n"
    self.mainStatusSet = True
  def configurationFileSaveDef(self):
    ret = QtGui.QMessageBox.Save
    if (not self.con.isOpen()):
      msgBox = QtGui.QMessageBox(mymw)
      msgBox.setText("The robot is not connected and some configuration values may have default value!")
      msgBox.setInformativeText("Do you want to save all parameters to regbot.ini anyhow?")
      msgBox.setStandardButtons(QtGui.QMessageBox.Save | QtGui.QMessageBox.Cancel)
      msgBox.setDefaultButton(QtGui.QMessageBox.Save)
      ret = msgBox.exec_()
    if ret == QtGui.QMessageBox.Save:
      # Save was clicked
      self.configurationFileSave('regbot.ini', True)    
  def configurationFileLoadDef(self, bootload):
    self.configurationFileLoad('regbot.ini', bootload)
  def configurationFileSaveAs(self):
    filename = QtGui.QFileDialog.getOpenFileName(mymw,'save configuration as', os.getcwd(), 'data (*.ini)')
    if (filename is not None  and len(filename) > 0):
      self.configurationFileSave(filename, True)
      print("saved config to " + filename)
  def configurationFileLoadFrom(self):
    filename = QtGui.QFileDialog.getOpenFileName(mymw,'configuration file to open', '', 'data (*.ini)')
    if (filename is not None and len(filename) > 0):
      self.configurationFileLoad(filename, False)
      print("Loaded configuration from " + filename)
  def configurationFileSave(self, filename, verbose):
    # create configuration to save
    config = ConfigParser.SafeConfigParser()
    config.add_section('main')
    config.set('main', 'version', self.clientVersion())
    config.set('main', 'mission', str(mymw.ui.main_mission_2.value()))
    config.set('main', 'statusInterval', str(mymw.ui.main_push_interval.value()))
    config.set('main', 'usbDeviceName', str(mymw.ui.usb_device_name.text()))
    config.set('main', 'connect', str(mymw.ui.connect_usb.isChecked()))
    # wifi connect
    config.set('main', 'wifiHost', str(mymw.ui.wifi_host_name.text()))
    config.set('main', 'wifiConnect', str(mymw.ui.connect_wifi.isChecked()))
    config.set('main', 'wifiport', str(mymw.ui.wifi_port.text()))
    # show tabs
    config.add_section('show')
    config.set('show', 'debug', str(mymw.ui.actionDebug.isChecked()))
    config.set('show', 'Z', str(mymw.ui.actionZ.isChecked()))
    config.set('show', 'wifi', str(mymw.ui.actionWifi.isChecked()))
    config.set('show', 'Log', str(mymw.ui.actionLog.isChecked()))
    config.set('show', 'Robot', str(mymw.ui.actionRobot.isChecked()))
    config.set('show', 'edge', str(mymw.ui.actionLine.isChecked()))
    config.set('show', 'distance', str(mymw.ui.actionDistance.isChecked()))
    config.set('show', 'IMU', str(mymw.ui.actionIMU.isChecked()))
    config.set('show', 'velocity', str(mymw.ui.actionVelocity.isChecked()))
    config.set('show', 'position', str(mymw.ui.actionPosition.isChecked()))
    config.set('show', 'turn', str(mymw.ui.actionTurn.isChecked()))
    config.set('show', 'balance', str(mymw.ui.actionBalance.isChecked()))
    config.set('show', 'ss', str(mymw.ui.actionSS.isChecked()))
    config.set('show', 'follow_edge', str(mymw.ui.actionFollow_line.isChecked()))
    config.set('show', 'follow_wall', str(mymw.ui.actionFollow_wall.isChecked()))
    config.set('show', 'mission', str(mymw.ui.actionMission.isChecked()))
    # log settings
    config.add_section('log')
    config.set('log', 'interval', str(mymw.ui.log_interval.value()))
    config.set('log', 'mission_state', str(mymw.ui.log_lms.isChecked()))
    config.set('log', 'Acceleration', str(mymw.ui.log_lac.isChecked()))
    config.set('log', 'Gyro', str(mymw.ui.log_lgy.isChecked()))
    config.set('log', 'Encoder', str(mymw.ui.log_lme.isChecked()))
    config.set('log', 'motorRef', str(mymw.ui.log_lvr.isChecked()))
    config.set('log', 'motorVoltage', str(mymw.ui.log_lmv.isChecked()))
    config.set('log', 'motorCurrent', str(mymw.ui.log_lma.isChecked()))
    config.set('log', 'wheelVelocity', str(mymw.ui.log_lmr.isChecked()))
    config.set('log', 'robotPose', str(mymw.ui.log_lpo.isChecked()))
    config.set('log', 'lineSensor', str(mymw.ui.log_line.isChecked()))
    config.set('log', 'irdist', str(mymw.ui.log_distance.isChecked()))
    config.set('log', 'turnrate', str(mymw.ui.log_turn_rate.isChecked()))
    config.set('log', 'batteryVoltage', str(mymw.ui.log_lbt.isChecked()))
    config.set('log', 'balanceCtrl', str(mymw.ui.log_lbc.isChecked()))
    config.set('log', 'controlTime', str(mymw.ui.log_lct.isChecked()))
    config.set('log', 'extraInfo', str(mymw.ui.log_lex.isChecked()))
    config.set('log', 'logFilename', str(mymw.ui.log_filename.text()))
    config.set('log', 'allow', str(mymw.ui.log_allow.isChecked()))
    # velocity control
    config.add_section('wheelVel')
    config.set('wheelVel', 'regUse', str(mymw.ui.reg_vel_use.isChecked()))
    config.set('wheelVel', 'useVelEst', str(mymw.ui.reg_vel_est_use.isChecked()))
    config.set('wheelVel', 'useVelLeadFwd', str(mymw.ui.reg_vel_LeadFwd.isChecked()))
    config.set('wheelVel', 'accLimit', str(mymw.ui.reg_vel_acc_limit.value()))
    config.set('wheelVel', 'kp', str(mymw.ui.reg_vel_KP.value()))
    config.set('wheelVel', 'tauI', str(mymw.ui.reg_vel_tau_i.value()))
    config.set('wheelVel', 'tauD', str(mymw.ui.reg_vel_tau_d.value()))
    config.set('wheelVel', 'alpha', str(mymw.ui.reg_vel_alpha.value()))
    config.set('wheelVel', 'imax', str(mymw.ui.reg_vel_integrate_max.value()))
    config.set('wheelVel', 'stepTime', str(mymw.ui.reg_vel_steptime.value()))
    config.set('wheelVel', 'stepFromReg', str(self.regVel.regStepFromReg))  #mymw.ui.reg_vel_step_from.value()))
    config.set('wheelVel', 'stepToReg', str(self.regVel.regStepToReg)) #mymw.ui.reg_vel_step_to.value()))
    config.set('wheelVel', 'stepFromV', str(self.regVel.regStepFromV))  #mymw.ui.reg_vel_step_from.value()))
    config.set('wheelVel', 'stepToV', str(self.regVel.regStepToV)) #mymw.ui.reg_vel_step_to.value()))
    config.set('wheelVel','motorVoltLimit', str(mymw.ui.reg_vel_volt_limit.value()))
    # position control
    config.add_section('Position')
    config.set('Position', 'regUse', str(mymw.ui.reg_pos_use.isChecked()))
    config.set('Position', 'LeadFwd', str(mymw.ui.reg_pos_LeadFwd.isChecked()))
    config.set('Position', 'kp', str(mymw.ui.reg_pos_KP.value()))
    config.set('Position', 'tauI', str(mymw.ui.reg_pos_tau_i.value()))
    config.set('Position', 'tauD', str(mymw.ui.reg_pos_tau_d.value()))
    config.set('Position', 'alpha', str(mymw.ui.reg_pos_alpha.value()))
    config.set('Position', 'imax', str(mymw.ui.reg_pos_ilimit.value()))
    config.set('Position', 'stepTime', str(mymw.ui.reg_pos_step_time.value()))
    config.set('Position', 'stepFrom', str(mymw.ui.reg_pos_step_from.value()))
    config.set('Position', 'stepTo', str(mymw.ui.reg_pos_step_to.value()))
    config.set('Position', 'outmax', str(mymw.ui.reg_pos_out_limit.value()))
    # turn control
    config.add_section('turn')
    config.set('turn', 'regUse', str(mymw.ui.reg_turn_use.isChecked()))
    config.set('turn', 'LeadFwd', str(mymw.ui.reg_turn_LeadFwd.isChecked()))
    config.set('turn', 'kp', str(mymw.ui.reg_turn_KP.value()))
    config.set('turn', 'tauI', str(mymw.ui.reg_turn_tau_i.value()))
    config.set('turn', 'tauD', str(mymw.ui.reg_turn_tau_d.value()))
    config.set('turn', 'alpha', str(mymw.ui.reg_turn_alpha.value()))
    config.set('turn', 'imax', str(mymw.ui.reg_turn_ilimit.value()))
    config.set('turn', 'outmax', str(mymw.ui.reg_turn_out_limit.value()))
    config.set('turn', 'stepOnTime', str(mymw.ui.reg_turn_step_on.value()))
    config.set('turn', 'stepOffTime', str(mymw.ui.reg_turn_step_off.value()))
    config.set('turn', 'stepValueReg', str(self.regTurn.stepValueTurnReg))  # mymw.ui.reg_turn_step_val.value()))
    config.set('turn', 'stepValueMotReg', str(self.regTurn.stepValueMotorReg))  # mymw.ui.reg_turn_step_val.value()))
    config.set('turn', 'stepValueMotV', str(self.regTurn.stepValueMotorV))  # mymw.ui.reg_turn_step_val.value()))
    config.set('turn', 'stepVelocityMotReg', str(self.regTurn.velBaseMotorReg))  #mymw.ui.reg_turn_step_on.value()))
    config.set('turn', 'stepVelocityMotV', str(self.regTurn.velBaseMotorV))  #mymw.ui.reg_turn_step_on.value()))
    # mission
    config.add_section('mission')
    config.set('mission', 'mission', str(mymw.ui.mission_edit.toPlainText()))
    config.set('mission', 'filename', str(mymw.ui.mission_filename.text()))
    # balance control
    config.add_section('balance')
    config.set('balance', 'Use', str(mymw.ui.reg_bal_use.isChecked()))
    config.set('balance', 'Kp', str(mymw.ui.reg_bal_KP.value()))
    config.set('balance', 'tauI', str(mymw.ui.reg_bal_tau_i.value()))
    config.set('balance', 'tauD', str(mymw.ui.reg_bal_tau_d.value()))
    config.set('balance', 'alpha', str(mymw.ui.reg_bal_alpha.value()))
    config.set('balance', 'imax', str(mymw.ui.reg_bal_integrate_max.value()))
    config.set('balance', 'outmax', str(mymw.ui.reg_bal_out_limit.value()))
    config.set('balance', 'leadFwd', str(mymw.ui.reg_bal_LeadFwd.isChecked()))
    config.set('balance', 'leadGyro', str(mymw.ui.reg_bal_LeadGyro.isChecked()))
    # balance velocity
    config.add_section('balanceVel')
    config.set('balanceVel', 'Use', str(mymw.ui.reg_balvel_use.isChecked()))
    config.set('balanceVel', 'Kp', str(mymw.ui.reg_mvel_KP.value()))
    config.set('balanceVel', 'tauI', str(mymw.ui.reg_mvel_tau_i.value()))
    config.set('balanceVel', 'tauD', str(mymw.ui.reg_mvel_tau_d.value()))
    config.set('balanceVel', 'alpha', str(mymw.ui.reg_mvel_alpha.value()))
    config.set('balanceVel', 'zeta', str(mymw.ui.reg_mvel_zeta.value()))
    config.set('balanceVel', 'imax', str(mymw.ui.reg_mvel_integrate_max.value()))
    config.set('balanceVel', 'leadFwd', str(mymw.ui.reg_mvel_LeadFwd.isChecked()))
    config.set('balanceVel', 'outmax', str(mymw.ui.reg_mvel_out_limit.value()))
    config.set('balanceVel', 'stepTime', str(mymw.ui.reg_bal_steptime.value()))
    config.set('balanceVel', 'stepFrom', str(self.regBal.regStepFrom))
    config.set('balanceVel', 'stepTo', str(self.regBal.regStepTo))
    config.set('balanceVel', 'stepFromVel', str(self.regBal.regStepVFrom))
    config.set('balanceVel', 'stepToVel', str(self.regBal.regStepVTo))
    #config.set('balanceVel', 'stepFromRadSec', str(self.regBal.regStepRSFrom))
    #config.set('balanceVel', 'stepToRadSec', str(self.regBal.regStepRSTo))
    config.set('balanceVel', 'stepFromMSec', str(self.regBal.regStepMSFrom))
    config.set('balanceVel', 'stepToMSec', str(self.regBal.regStepMSTo))
    config.set('balanceVel', 'stepFromMotVolt', str(self.regBal.regStepMVFrom))
    config.set('balanceVel', 'stepToMotVolt', str(self.regBal.regStepMVTo))
    # balance velocity
    config.add_section('balanceSS')
    config.set('balanceSS', 'Use', str(mymw.ui.reg_bal_ss_use.isChecked()))
    config.set('balanceSS', 'K_tilt', str(mymw.ui.reg_bal_ss_k_tilt.value()))
    config.set('balanceSS', 'K_gyro', str(mymw.ui.reg_bal_ss_k_gyro.value()))
    config.set('balanceSS', 'K_pos', str(mymw.ui.reg_bal_ss_k_pos.value()))
    config.set('balanceSS', 'K_vel', str(mymw.ui.reg_bal_ss_k_vel.value()))
    config.set('balanceSS', 'K_motor', str(mymw.ui.reg_bal_ss_k_motor.value()))
    config.set('balanceSS', 'outmax', str(mymw.ui.reg_bal_ss_out_limit.value()))
    config.set('balanceSS', 'stepTime', str(mymw.ui.reg_bal_ss_steptime.value()))
    config.set('balanceSS', 'stepFrom', str(mymw.ui.reg_bal_ss_step_from.value()))
    config.set('balanceSS', 'stepTo', str(mymw.ui.reg_bal_ss_step_to.value()))
    config.set('balanceSS', 'stepPos', str(mymw.ui.reg_bal_ss_step.value()))
    # line sensor page
    config.add_section('lineSensor')
    config.set('lineSensor', 'maximum', str(mymw.ui.line_disp_max_value.value()))
    config.set('lineSensor', 'use', str(mymw.ui.ls_use_sensor.isChecked()))
    config.set('lineSensor', 'white', str(mymw.ui.ls_line_white.isChecked()))
    #config.set('lineSensor', 'regUse', str(mymw.ui.reg_turn_use_2.isChecked()))
    config.set('lineSensor', 'LeadFwd', str(mymw.ui.reg_turn_LeadFwd_2.isChecked()))
    config.set('lineSensor', 'kp', str(mymw.ui.reg_turn_KP_2.value()))
    config.set('lineSensor', 'tauI', str(mymw.ui.reg_turn_tau_i_2.value()))
    config.set('lineSensor', 'tauD', str(mymw.ui.reg_turn_tau_d_2.value()))
    config.set('lineSensor', 'alpha', str(mymw.ui.reg_turn_alpha_2.value()))
    config.set('lineSensor', 'imax', str(mymw.ui.reg_turn_ilimit_2.value()))
    config.set('lineSensor', 'outmax', str(mymw.ui.reg_turn_out_limit_2.value()))
    config.set('lineSensor', 'stepOnTime', str(mymw.ui.reg_turn_step_on_2.value()))
    config.set('lineSensor', 'stepFrom', str(mymw.ui.reg_turn_step_from_2.value()))
    config.set('lineSensor', 'stepTo', str(mymw.ui.reg_turn_step_val_2.value()))
    config.set('lineSensor', 'baseVel', str(mymw.ui.reg_turn_step_vel_2.value()))
    config.set('lineSensor', 'LEDpower', str(mymw.ui.ls_power_high.isChecked()))
    config.set('lineSensor', 'LEDpowerAuto', str(mymw.ui.ls_power_auto.isChecked()))
    # wall follow
    config.add_section('wall')
    config.set('wall', 'use', str(mymw.ui.wall_sensor_on.isChecked()))
    config.set('wall', 'leftsensor', str(mymw.ui.wall_follow_left.isChecked()))
    config.set('wall', 'LeadFwd', str(mymw.ui.reg_wall_LeadFwd.isChecked()))
    config.set('wall', 'kp', str(mymw.ui.reg_wall_KP.value()))
    config.set('wall', 'tauI', str(mymw.ui.reg_wall_tau_i.value()))
    config.set('wall', 'tauD', str(mymw.ui.reg_wall_tau_d.value()))
    config.set('wall', 'alpha', str(mymw.ui.reg_wall_alpha.value()))
    config.set('wall', 'imax', str(mymw.ui.reg_wall_ilimit.value()))
    config.set('wall', 'outmax', str(mymw.ui.reg_wall_out_limit.value()))
    config.set('wall', 'stepTime', str(mymw.ui.reg_wall_step_time.value()))
    config.set('wall', 'stepFrom', str(mymw.ui.reg_wall_step_from.value()))
    config.set('wall', 'stepTo', str(mymw.ui.reg_wall_step_to.value()))
    config.set('wall', 'baseVel', str(mymw.ui.reg_wall_step_vel.value()))
    # robot specifics
    for rb in robot.info.robots:
      print("List robot ID's - now ID (" + str(rb.robotID) + ") " + rb.name)
    for rb in robot.info.robots:
      idstr = "robot" + str(rb.robotID)
      if rb.robotID == int(mymw.ui.robot_id.value()) and mymw.ui.save_id_on_robot.isEnabled():
        # some data may have changed
        rb.wheelbase =  mymw.ui.robot_base.value()
        rb.gear = mymw.ui.robot_gear.value()
        rb.pulsePerRev = mymw.ui.robot_pulse_per_rev.value()
        rb.wheelLRadius = mymw.ui.robot_wheel_radius_left.value()
        rb.wheelRRadius = mymw.ui.robot_wheel_radius_right.value()
        rb.balanceOffset = mymw.ui.robot_balance_offset.value()
        rb.batteryUse = mymw.ui.robot_on_battery.isChecked()
        rb.batteryIdleVolt = mymw.ui.robot_battery_idle_volt.value()
        rb.wifiIP = mymw.ui.wifi_ip.text()
        rb.wifiMAC = mymw.ui.wifi_mac.text()
        #rb.wifiport = mymw.ui.wifi_port.text()
        #print("save values on screen")
      #
      #print("Adding section for " + idstr + ", wr1: " + str(rb.wheelRRadius))
      try:
        config.add_section(idstr)
      except:
        print("error - exist already: " + idstr)
        pass
      config.set(idstr, 'name', rb.name)
      config.set(idstr, 'version', rb.version)
      config.set(idstr, 'robotBase', str(rb.wheelbase))
      #print("writeing wheelbase = " + str(rb.wheelbase) + " robot " + str(rb.robotID))
      config.set(idstr, 'gear', str(rb.gear))
      config.set(idstr, 'encoderPPR', str(rb.pulsePerRev))
      config.set(idstr, 'wheelRadiusLeft', str(rb.wheelLRadius))
      #print("writeing wheelradius 0 = " + str(rb.wheelLRadius) + " robot " + str(rb.robotID))
      config.set(idstr, 'wheelRadiusRight', str(rb.wheelRRadius))
      #print("writeing wheelradius 1 = " + str(rb.wheelRRadius) + " robot " + str(rb.robotID))
      config.set(idstr, 'balanceOffset', str(rb.balanceOffset))
      config.set(idstr, 'batteryUse', str(rb.batteryUse))
      config.set(idstr, 'batteryIdleVolt', str(rb.batteryIdleVolt))
      config.set(idstr, 'wifiIP', str(rb.wifiIP))
      config.set(idstr, 'wifiMAC', str(rb.wifiMAC))
      #config.set(idstr, 'wifiport', str(rb.wifiport))
      pass
    # override with current robot
    #if (mymw.ui.robot_id.value() > 0):
      #idstr = "robot" + str(int(mymw.ui.robot_id.value()))
      #config.add_section(idstr)
      #config.set(idstr, 'version', str(self.info.version))
      #config.set(idstr, 'robotBase', str(mymw.ui.robot_base.value()))
      #config.set(idstr, 'gear', str(mymw.ui.robot_gear.value()))
      #config.set(idstr, 'encoderPPR', str(mymw.ui.robot_pulse_per_rev.value()))
      #config.set(idstr, 'weelRadiusLeft', str(mymw.ui.robot_wheel_radius_left.value()))
      #config.set(idstr, 'weelRadiusRight', str(mymw.ui.robot_wheel_radius_right.value()))
      #config.set(idstr, 'balanceOffset', str(mymw.ui.robot_balance_offset.value()))
    # rename old file first
    if os.path.exists(filename):
      newname = 'regbot_' + time.strftime("%Y%2m%2d_%02H%2M%2S") + '.ini'
      #t = time.gmtime()
      #newname = 'regbot_' + str(t.tm_year) + str(t.tm_mon) + str(t.tm_mday) + "_" + str(t.tm_hour) + str(t.tm_min) + str(t.tm_sec) + '.ini'
      if (verbose):
        print("Renamed " + filename + " to " + newname)
      os.rename(filename, newname)
    # save new file
    with open(filename, 'wb') as configFile:
      config.write(configFile)
      if (verbose):
        print("Created new " + filename)
    #finished
    pass
  #
  # Load configuration from configuration file
  #
  def configurationFileLoad(self, filename, bootload):
    config = ConfigParser.SafeConfigParser()
    config.read(filename)
    print("config - reading " + filename)
    try:
      mymw.ui.main_mission_2.setValue(config.getint('main', 'mission'))
      mymw.ui.main_push_interval.setValue(config.getint('main', 'statusInterval'))
      mymw.ui.usb_device_name.setText(config.get('main', 'usbDeviceName'))
      mymw.ui.connect_usb.setChecked(config.getboolean('main', 'connect'))
      robot.connect_usb_changed()
      try:
        mymw.ui.wifi_host_name.setText(config.get('main', 'wifiHost'))
        mymw.ui.connect_wifi.setChecked(config.getboolean('main', 'wifiConnect'))
        mymw.ui.wifi_port.setText(config.get('main','wifiport'))
        robot.connect_wifi_changed()
      except:
        pass
      #
      if bootload:
        # load visible tab list during boot only
        mymw.ui.actionDebug.setChecked(config.getboolean('show', 'debug'))
        mymw.ui.actionZ.setChecked(config.getboolean('show', 'Z'))
        mymw.ui.actionWifi.setChecked(config.getboolean('show', 'wifi'))
        mymw.ui.actionLog.setChecked(config.getboolean('show', 'Log'))
        mymw.ui.actionRobot.setChecked(config.getboolean('show', 'Robot'))
        mymw.ui.actionIMU.setChecked(config.getboolean('show', 'IMU'))
        mymw.ui.actionLine.setChecked(config.getboolean('show', 'edge'))
        mymw.ui.actionDistance.setChecked(config.getboolean('show', 'distance'))
        mymw.ui.actionVelocity.setChecked(config.getboolean('show', 'velocity'))
        mymw.ui.actionPosition.setChecked(config.getboolean('show', 'position'))
        mymw.ui.actionTurn.setChecked(config.getboolean('show', 'turn'))
        mymw.ui.actionBalance.setChecked(config.getboolean('show', 'balance'))
        mymw.ui.actionSS.setChecked(config.getboolean('show', 'SS'))
        mymw.ui.actionFollow_line.setChecked(config.getboolean('show', 'follow_edge'))
        mymw.ui.actionFollow_wall.setChecked(config.getboolean('show', 'follow_wall'))
        mymw.ui.actionMission.setChecked(config.getboolean('show', 'mission'))
        # implement config actions
        mymw.menuShowDebug()
        mymw.menuShowZ()
        mymw.menuShowLog()
        mymw.menuShowRobot()
        mymw.menuShowIMU()
        mymw.menuShowVelocity()
        mymw.menuShowPosition()
        mymw.menuShowLine()
        mymw.menuShowDist()
        mymw.menuShowTurn()
        mymw.menuShowBalance()
        mymw.menuShowSS()
        mymw.menuShowFollowLine()
        mymw.menuShowFollowWall()
        mymw.menuShowMission()
      #print("log")
      # log options
      mymw.ui.log_interval.setValue(config.getint('log', 'interval'))
      mymw.ui.log_lms.setChecked(config.getboolean('log', 'mission_state'))
      mymw.ui.log_lac.setChecked(config.getboolean('log', 'Acceleration'))
      mymw.ui.log_lgy.setChecked(config.getboolean('log', 'gyro'))
      mymw.ui.log_lme.setChecked(config.getboolean('log', 'encoder'))
      mymw.ui.log_lvr.setChecked(config.getboolean('log', 'motorRef'))
      mymw.ui.log_lmv.setChecked(config.getboolean('log', 'motorVoltage'))
      mymw.ui.log_lma.setChecked(config.getboolean('log', 'motorCurrent'))
      mymw.ui.log_lmr.setChecked(config.getboolean('log', 'wheelVelocity'))
      mymw.ui.log_lpo.setChecked(config.getboolean('log', 'robotPose'))
      mymw.ui.log_line.setChecked(config.getboolean('log', 'lineSensor'))
      mymw.ui.log_distance.setChecked(config.getboolean('log', 'irdist'))
      mymw.ui.log_turn_rate.setChecked(config.getboolean('log', 'turnrate'))
      mymw.ui.log_lbt.setChecked(config.getboolean('log', 'batteryVoltage'))
      mymw.ui.log_lbc.setChecked(config.getboolean('log', 'balanceCtrl'))
      mymw.ui.log_lct.setChecked(config.getboolean('log', 'controlTime'))
      mymw.ui.log_lex.setChecked(config.getboolean('log', 'extraInfo'))
      mymw.ui.log_filename.setText(config.get('log', 'logFilename'))
      mymw.ui.log_allow.setChecked(config.getboolean('log', 'allow'))
      #print("vel")
      # reg vel
      mymw.ui.reg_vel_use.setChecked(config.getboolean('wheelVel', 'regUse'))
      mymw.ui.reg_vel_est_use.setChecked(config.getboolean('wheelVel', 'useVelEst'))
      mymw.ui.reg_vel_LeadFwd.setChecked(config.getboolean('wheelVel', 'useVelLeadFwd'))
      mymw.ui.reg_vel_KP.setValue(config.getfloat('wheelVel', 'kp'))
      mymw.ui.reg_vel_tau_i.setValue(config.getfloat('wheelVel', 'tauI'))
      mymw.ui.reg_vel_tau_d.setValue(config.getfloat('wheelVel', 'tauD'))
      mymw.ui.reg_vel_alpha.setValue(config.getfloat('wheelVel', 'alpha'))
      mymw.ui.reg_vel_integrate_max.setValue(config.getfloat('wheelVel', 'imax'))
      mymw.ui.reg_vel_steptime.setValue(config.getfloat('wheelVel', 'stepTime'))
      self.regVel.regStepFromReg = config.getfloat('wheelVel', 'stepFromReg')
      self.regVel.regStepToReg = config.getfloat('wheelVel', 'steptoreg')
      self.regVel.regStepFromV = config.getfloat('wheelVel', 'stepFromV')
      self.regVel.regStepToV = config.getfloat('wheelVel', 'steptov')
      #print("pre-hep")
      #mymw.ui.reg_vel_help.setPlainText(config.get('wheelVel', 'help'))
      mymw.ui.reg_vel_volt_limit.setValue(config.getfloat('wheelVel','motorVoltLimit'))
      self.regVel.configChanged()
      if self.isConnected():
        self.regulatorParamVelApply()
      #
      #config.set('Position', 'regUse', str(mymw.ui.reg_pos_use.isChecked()))
      #config.set('Position', 'LeadFwd', str(mymw.ui.reg_pos_LeadFwd.isChecked()))
      #config.set('Position', 'kp', str(mymw.ui.reg_pos_KP.value()))
      #config.set('Position', 'tauI', str(mymw.ui.reg_pos_tau_i.value()))
      #config.set('Position', 'tauD', str(mymw.ui.reg_pos_tau_d.value()))
      #config.set('Position', 'alpha', str(mymw.ui.reg_pos_alpha.value()))
      #config.set('Position', 'imax', str(mymw.ui.reg_pos_ilimit.value()))
      #config.set('Position', 'stepTime', str(mymw.ui.reg_pos_step_time.value()))
      #config.set('Position', 'stepFrom', str(mymw.ui.reg_pos_step_from))
      #config.set('Position', 'stepTo', str(mymw.ui.reg_pos_step_to))
      #config.set('Position', 'outmax', str(mymw.ui.reg_pos_out_limit.value()))
      #print("turn")
      #position
      mymw.ui.reg_pos_use.setChecked(config.getboolean('Position', 'regUse'))
      mymw.ui.reg_pos_LeadFwd.setChecked(config.getboolean('Position', 'LeadFwd'))
      mymw.ui.reg_pos_KP.setValue(config.getfloat('Position', 'kp'))
      mymw.ui.reg_pos_tau_i.setValue(config.getfloat('Position', 'tauI'))
      mymw.ui.reg_pos_tau_d.setValue(config.getfloat('Position', 'tauD'))
      mymw.ui.reg_pos_alpha.setValue(config.getfloat('Position', 'alpha'))
      mymw.ui.reg_pos_ilimit.setValue(config.getfloat('Position', 'imax'))
      mymw.ui.reg_pos_out_limit.setValue(config.getfloat('Position', 'outmax'))
      mymw.ui.reg_pos_step_time.setValue(config.getfloat('Position', 'stepTime'))
      mymw.ui.reg_pos_step_from.setValue(config.getfloat('Position', 'stepFrom'))       
      mymw.ui.reg_pos_step_to.setValue(config.getfloat('Position', 'stepTo'))       
      if self.isConnected():
        self.regPosition.regulatorParamApply()
      #reg turn
      mymw.ui.reg_turn_use.setChecked(config.getboolean('turn', 'regUse'))
      mymw.ui.reg_turn_LeadFwd.setChecked(config.getboolean('turn', 'LeadFwd'))
      mymw.ui.reg_turn_KP.setValue(config.getfloat('turn', 'kp'))
      mymw.ui.reg_turn_tau_i.setValue(config.getfloat('turn', 'tauI'))
      mymw.ui.reg_turn_tau_d.setValue(config.getfloat('turn', 'tauD'))
      mymw.ui.reg_turn_alpha.setValue(config.getfloat('turn', 'alpha'))
      mymw.ui.reg_turn_ilimit.setValue(config.getfloat('turn', 'imax'))
      mymw.ui.reg_turn_out_limit.setValue(config.getfloat('turn', 'outmax'))
      mymw.ui.reg_turn_step_on.setValue(config.getfloat('turn', 'stepOnTime'))
      mymw.ui.reg_turn_step_off.setValue(config.getfloat('turn', 'stepOffTime'))
      # turn values depend on state
      self.regTurn.stepValueTurnReg = config.getfloat('turn', 'stepValueReg')
      self.regTurn.stepValueMotorReg = config.getfloat('turn', 'stepValueMotReg')
      self.regTurn.stepValueMotorV = config.getfloat('turn', 'stepValueMotV')
      #mymw.ui.reg_turn_step_val.setValue(config.getfloat('turn', 'stepValue'))
      self.regTurn.velBaseMotorReg = config.getfloat('turn', 'stepVelocityMotReg')
      self.regTurn.velBaseMotorV = config.getfloat('turn', 'stepVelocityMotV')
      #mymw.ui.reg_turn_step_vel.setValue(config.getfloat('turn', 'stepVelocity'))
      self.regTurn.configChanged()
      if self.isConnected():
        self.regulatorParamTurnApply()

      #mymw.ui.reg_turn_help.setPlainText(config.get('turn', 'help'))
      #
      #print("mission start")
      # mission
      m = config.get('mission', 'mission')
      indent = ""
      for ml in m.splitlines():
        if ml[0] == '#':
          self.mission.missionTextEdit += indent + ml + '\n'
        elif  "thread" in ml:
          self.mission.missionTextEdit += ml + '\n'
          indent = "    "
        else:
          self.mission.missionTextEdit += indent + ml + '\n'
      self.mission.missionTextEditChaged = True
      #mymw.ui.mission_help.setPlainText(config.get('mission', 'help'))
      #if self.isConnected():
        #self.mission.sendToRobot()
      mymw.ui.mission_filename.setText(config.get('mission', 'filename'))
      print("mission load end")
      # reg balance
      #print("bal")
      mymw.ui.reg_bal_use.setChecked(config.getboolean('balance', 'Use'))
      mymw.ui.reg_bal_KP.setValue(config.getfloat('balance', 'kp'))
      mymw.ui.reg_bal_tau_i.setValue(config.getfloat('balance', 'tauI'))
      mymw.ui.reg_bal_tau_d.setValue(config.getfloat('balance', 'tauD'))
      mymw.ui.reg_bal_alpha.setValue(config.getfloat('balance', 'alpha'))
      mymw.ui.reg_bal_integrate_max.setValue(config.getfloat('balance', 'imax'))
      mymw.ui.reg_bal_LeadFwd.setChecked(config.getboolean('balance', 'leadFwd'))
      mymw.ui.reg_bal_LeadGyro.setChecked(config.getboolean('balance', 'leadGyro'))
      mymw.ui.reg_bal_out_limit.setValue(config.getfloat('balance', 'outmax'))
      # reg balance velocity
      #print("balvel")
      mymw.ui.reg_balvel_use.setChecked(config.getboolean('balanceVel', 'Use'))
      mymw.ui.reg_mvel_LeadFwd.setChecked(config.getboolean('balanceVel', 'leadFwd'))
      mymw.ui.reg_mvel_KP.setValue(config.getfloat('balanceVel', 'kp'))
      mymw.ui.reg_mvel_tau_i.setValue(config.getfloat('balanceVel', 'tauI'))
      mymw.ui.reg_mvel_tau_d.setValue(config.getfloat('balanceVel', 'tauD'))
      mymw.ui.reg_mvel_alpha.setValue(config.getfloat('balanceVel', 'alpha'))
      mymw.ui.reg_mvel_zeta.setValue(config.getfloat('balanceVel', 'zeta'))
      mymw.ui.reg_mvel_integrate_max.setValue(config.getfloat('balanceVel', 'imax'))
      mymw.ui.reg_mvel_out_limit.setValue(config.getfloat('balanceVel', 'outmax'))
      mymw.ui.reg_bal_steptime.setValue(config.getfloat('balanceVel', 'stepTime'))
      self.regBal.regStepFrom = config.getfloat('balanceVel', 'stepFrom')
      self.regBal.regStepTo = config.getfloat('balanceVel', 'stepTo')
      self.regBal.regStepVFrom = config.getfloat('balanceVel', 'stepFromVel')
      self.regBal.regStepVTo = config.getfloat('balanceVel', 'stepToVel')
      #self.regBal.regStepRSFrom = config.getfloat('balanceVel', 'stepFromRadSec')
      #self.regBal.regStepRSTo = config.getfloat('balanceVel', 'stepToRadSec')
      self.regBal.regStepMSFrom = config.getfloat('balanceVel', 'stepFromMSec')
      self.regBal.regStepMSTo = config.getfloat('balanceVel', 'stepToMSec')
      self.regBal.regStepMVFrom = config.getfloat('balanceVel', 'stepFromMotVolt')
      self.regBal.regStepMVTo = config.getfloat('balanceVel', 'stepToMotVolt')
      self.regBal.configChanged()
      if self.isConnected():
        self.regulatorParamBalApply()
      # State space config
      #print("bal SS config read")
      mymw.ui.reg_bal_ss_use.setChecked(config.getboolean('balanceSS', 'Use'))
      mymw.ui.reg_bal_ss_k_tilt.setValue(config.getfloat('balanceSS', 'K_tilt'))
      mymw.ui.reg_bal_ss_k_gyro.setValue(config.getfloat('balanceSS', 'K_gyro'))
      mymw.ui.reg_bal_ss_k_pos.setValue(config.getfloat('balanceSS', 'K_pos'))
      mymw.ui.reg_bal_ss_k_vel.setValue(config.getfloat('balanceSS', 'K_vel'))
      mymw.ui.reg_bal_ss_k_motor.setValue(config.getfloat('balanceSS', 'K_motor'))
      mymw.ui.reg_bal_ss_out_limit.setValue(config.getfloat('balanceSS', 'outmax'))
      mymw.ui.reg_bal_ss_steptime.setValue(config.getfloat('balanceSS', 'stepTime'))
      mymw.ui.reg_bal_ss_step_from.setValue(config.getfloat('balanceSS', 'stepFrom'))
      mymw.ui.reg_bal_ss_step_to.setValue(config.getfloat('balanceSS', 'stepTo'))
      mymw.ui.reg_bal_ss_step.setValue(config.getfloat('balanceSS', 'stepPos'))
      if self.isConnected():
        self.regulatorParamSSApply()
      #
      # line sensor page
      mymw.ui.line_disp_max_value.setValue(config.getfloat('lineSensor', 'maximum'))
      mymw.ui.ls_use_sensor.setChecked(config.getboolean('lineSensor', 'use'))
      mymw.ui.ls_line_white.setChecked(config.getboolean('lineSensor', 'white'))
      mymw.ui.reg_turn_LeadFwd_2.setChecked(config.getboolean('lineSensor', 'LeadFwd'))
      mymw.ui.reg_turn_KP_2.setValue(config.getfloat('lineSensor', 'kp'))
      mymw.ui.reg_turn_tau_i_2.setValue(config.getfloat('lineSensor', 'tauI'))
      mymw.ui.reg_turn_tau_d_2.setValue(config.getfloat('lineSensor', 'tauD'))
      mymw.ui.reg_turn_alpha_2.setValue(config.getfloat('lineSensor', 'alpha'))
      mymw.ui.reg_turn_ilimit_2.setValue(config.getfloat('lineSensor', 'imax'))
      mymw.ui.reg_turn_out_limit_2.setValue(config.getfloat('lineSensor', 'outmax'))
      mymw.ui.reg_turn_step_on_2.setValue(config.getfloat('lineSensor', 'stepOnTime'))
      mymw.ui.reg_turn_step_from_2.setValue(config.getfloat('lineSensor', 'stepfrom'))
      mymw.ui.reg_turn_step_val_2.setValue(config.getfloat('lineSensor', 'stepTo'))
      mymw.ui.reg_turn_step_vel_2.setValue(config.getfloat('lineSensor', 'baseVel'))
      mymw.ui.ls_power_high.setChecked(config.getboolean('lineSensor', 'LEDpower'))
      mymw.ui.ls_power_auto.setChecked(config.getboolean('lineSensor', 'LEDpowerAuto'))
      self.lineSensor.configChanged()
      if self.isConnected():
        self.lineSensor.applySettings()
      # Wall follow
      if True:
        mymw.ui.wall_sensor_on.setChecked(config.getboolean('wall', 'use'))
        mymw.ui.wall_follow_left.setChecked(config.getboolean('wall', 'leftsensor'))
        mymw.ui.reg_wall_LeadFwd.setChecked(config.getboolean('wall', 'LeadFwd'))
        mymw.ui.reg_wall_KP.setValue(config.getfloat('wall', 'kp'))
        mymw.ui.reg_wall_tau_i.setValue(config.getfloat('wall', 'tauI'))
        mymw.ui.reg_wall_tau_d.setValue(config.getfloat('wall', 'tauD'))
        mymw.ui.reg_wall_alpha.setValue(config.getfloat('wall', 'alpha'))
        mymw.ui.reg_wall_ilimit.setValue(config.getfloat('wall', 'imax'))
        mymw.ui.reg_wall_out_limit.setValue(config.getfloat('wall', 'outmax'))
        mymw.ui.reg_wall_step_time.setValue(config.getfloat('wall', 'stepTime'))
        mymw.ui.reg_wall_step_from.setValue(config.getfloat('wall', 'stepfrom'))
        mymw.ui.reg_wall_step_to.setValue(config.getfloat('wall', 'stepTo'))
        mymw.ui.reg_wall_step_vel.setValue(config.getfloat('wall', 'baseVel'))
        self.irDist.configChanged()
        if self.isConnected():
          self.irDist.paramApply()

    except:
      # default, if no regbot.ini file is available
      mymw.ui.actionDebug.setChecked(True)
      mymw.ui.actionZ.setChecked(False)
      mymw.ui.actionWifi.setChecked(False)
      mymw.ui.actionLog.setChecked(True)
      mymw.ui.actionRobot.setChecked(True)
      mymw.ui.actionIMU.setChecked(False)
      mymw.ui.actionLine.setChecked(False)
      mymw.ui.actionDistance.setChecked(False)
      mymw.ui.actionVelocity.setChecked(True)
      mymw.ui.actionPosition.setChecked(True)
      mymw.ui.actionTurn.setChecked(True)
      mymw.ui.actionBalance.setChecked(False)
      mymw.ui.actionSS.setChecked(False)
      mymw.ui.actionFollow_line.setChecked(False)
      mymw.ui.actionMission.setChecked(True)
      # implement config actions
      mymw.menuShowDebug()
      mymw.menuShowZ()
      mymw.menuShowLog()
      mymw.menuShowRobot()
      mymw.menuShowIMU()
      mymw.menuShowVelocity()
      mymw.menuShowPosition()
      mymw.menuShowLine()
      mymw.menuShowDist()
      mymw.menuShowTurn()
      mymw.menuShowBalance()
      mymw.menuShowSS()
      mymw.menuShowFollowLine()
      mymw.menuShowMission()
      print("Configuration load: failed to find regbot.ini or at least one value in the file")
    for rid in range(1, 64):
      #rb = URobotInfo()
      try:
        idstr = "robot" + str(rid)
        #print("robot '" + idstr + "' loading:")
        rbase = config.getfloat(idstr, "robotBase")
        # if section with this id is found make a robot
        rb = robot.info.getRobot(rid)
        rb.name = config.get(idstr, "name")
        rb.wheelbase = rbase
        rb.version = config.get(idstr, "version")
        rb.gear = config.getfloat(idstr, "gear")
        rb.pulsePerRev = config.getfloat(idstr, "encoderPPR")
        rb.wheelLRadius = config.getfloat(idstr, "wheelRadiusLeft")
        rb.wheelRRadius = config.getfloat(idstr, "wheelRadiusRight")
        rb.balanceOffset = config.getfloat(idstr, "balanceOffset")
        rb.batteryUse = config.getboolean(idstr, "batteryUse")
        rb.batteryIdleVolt = config.getfloat(idstr, "batteryIdleVolt")
        try:
          rb.wifiIP = config.get(idstr, "wifiIP")
        except:
          rb.wifiIP = "no_IP"
        try:
          rb.wifiMAC = config.get(idstr, "wifiMAC")
        except:
          rb.wifiMAC = "no_MAC"
        #try:
          #rb.wifiport = config.getint(idstr, "wifiport")
        #except:
          #rb.wifiport = 0
        #print("loaded data for robot " + str(rb.robotID))
        #print("    base: " + str(rb.wheelbase))
        #print("    gear: " + str(rb.gear))
        #print("    ppr : " + str(rb.pulsePerRev))
        #print("    wr 1: " + str(rb.wheelLRadius))
        #print("    wr 2: " + str(rb.wheelRRadius))
        #print("    offs: " + str(rb.balanceOffset))
        #print("RID = '" + str(rid) + " == " + str(int(mymw.ui.robot_id.value())))
        if rid == int(mymw.ui.robot_id.value()):
          mymw.ui.robot_base.setValue(rb.wheelbase)
          mymw.ui.robot_gear.setValue(rb.gear)
          mymw.ui.robot_pulse_per_rev.setValue(rb.pulsePerRev)
          mymw.ui.robot_wheel_radius_left.setValue(rb.wheelLRadius)
          mymw.ui.robot_wheel_radius_right.setValue(rb.wheelRRadius)
          mymw.ui.robot_balance_offset.setValue(rb.balanceOffset)
          mymw.ui.robot_on_battery.setChecked(rb.batteryUse)
          mymw.ui.robot_battery_idle_volt.setValue(rb.batteryIdleVolt)
          #print("saved to vidgets")
          if self.isConnected():
            self.robotIdApply()
        #for rb in robot.info.robots:
        #print("    loaded robots ID " + str(rb.robotID))
      except:
        pass
        #print("Configuration load: failed to find value in config file for this robot " + str(rid))

    pass    
  def configurationRobotSave(self):
    self.conWrite("eew\n")
  def clientVersion(self):
    gg = CLIENT_REV.split()
    return "2." + gg[2]
  def clientVersionDate(self):
    gg = CLIENT_REV.split()
    return gg[3]
# class URobot end        


# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////


class PaintSupport(QtGui.QWidget):
  def paintPlus(self, paint, x, y):
      paint.drawLine(QtCore.QPoint(x - 3, y), QtCore.QPoint(x + 3, y))
      paint.drawLine(QtCore.QPoint(x, y - 3), QtCore.QPoint(x, y + 3))
  def paintMinus(self, paint, x, y):
      paint.drawLine(QtCore.QPoint(x - 3, y), QtCore.QPoint(x + 3, y))
  def paintDot(self, paint, x, y):
      paint.setBrush(QtCore.Qt.black)
      paint.drawEllipse(QtCore.QPoint(x, y), 3, 3)
      paint.setBrush(QtCore.Qt.NoBrush)
  def paintArrow(self, paint, x, y, dir):
      dx1 = -5 # down
      dx2 = +5
      dy1 = -5
      dy2 = -5
      if (dir == 0): # right
        dx1 = -5
        dx2 = -5
        dy1 = -5
        dy2 = +5
      elif (dir == 1): # up
        dx1 = -5
        dx2 = +5
        dy1 = +5
        dy2 = +5
      elif (dir == 2): # left
        dx1 = +5
        dx2 = +5
        dy1 = -5
        dy2 = +5
      pts = [QtCore.QPoint(x,y), 
             QtCore.QPoint(x + dx1,y + dy1),
             QtCore.QPoint(x + dx2,y + dy2)]
      poly = QtGui.QPolygon(pts)
      paint.setBrush(QtCore.Qt.black)
      paint.drawPolygon(poly)
      paint.setBrush(QtCore.Qt.NoBrush)
  
class UsbPaintSpace(PaintSupport):
  def paintEvent(self, QPaintEvent):
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.darkGreen)
      if (robot.isConnected()):
        paint.setBrush(QtCore.Qt.green)
      else:
        paint.setBrush(QtCore.Qt.NoBrush)
      paint.drawRect(1,1, mymw.ui.frame_usb_connect.width(), mymw.ui.frame_usb_connect.height())
      paint.setBrush(QtCore.Qt.NoBrush)
      print("usb frame painted")
      pass
      
class WifiPaintSpace(PaintSupport):
  def paintEvent(self, QPaintEvent):
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.darkBlue)
      if robot.wifiConnected and not robot.isConnected():
        # green if wifi is connected and USB is not
        paint.setBrush(QtCore.Qt.green)
      else:
        paint.setBrush(QtCore.Qt.NoBrush)
      paint.drawRect(1,1, mymw.ui.frame_wifi_connect.width(), mymw.ui.frame_wifi_connect.height())
      paint.setBrush(QtCore.Qt.NoBrush)
      print("wifi frame painted")
      pass

class VelocityPaintSpace(PaintSupport):
  # """My widget for drawing smth"""
  #def __init__(self):
      #super(PaintSpace, self).__init__()
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.darkBlue)
      #size = self.size()
      # sum point
      p1 = QtCore.QPoint(mymw.ui.frame_vel_step.x() + mymw.ui.frame_vel_step.width() + 20, 230)
      # step to u(t)
      # feedback sum circle
      paint.drawEllipse(p1, 14, 14)
      self.paintArrow(paint, p1.x() - 14, p1.y(), 0)
      self.paintArrow(paint, p1.x(), p1.y() + 14, 1)
      self.paintPlus(paint, p1.x() - 8, p1.y())
      self.paintMinus(paint, p1.x(), p1.y() + 8)
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), QtCore.QPoint(mymw.ui.frame_vel_reg.x(), p1.y())) # sum to reg
      if (mymw.ui.reg_vel_LeadFwd.isChecked()):
        # output of gear - low point
        p2 = QtCore.QPoint(mymw.ui.frame_vel_gear.x() + mymw.ui.frame_vel_gear.width() + 20, 
                          mymw.ui.frame_vel_reg.y() + mymw.ui.frame_vel_reg.height() + 34) # feed back - south-east point
      else:
        p2 = QtCore.QPoint(mymw.ui.frame_vel_gear.x() + mymw.ui.frame_vel_gear.width() + 20, 
                          mymw.ui.frame_vel_lead.y() + 20) # feed back - south-east point
      # u(t) join point just before limit
      p3 = QtCore.QPoint(mymw.ui.frame_vel_limit.x() - 20, mymw.ui.frame_vel_motor.y() + mymw.ui.frame_vel_motor.height()/2)
      paint.drawLine(p3, QtCore.QPoint(mymw.ui.frame_vel_limit.x(), p3.y())) # output line
      # output of gear (output
      paint.drawLine(QtCore.QPoint(p2.x() - 20, p3.y()), 
                     QtCore.QPoint(p2.x() + 20, p3.y())) # output line
      self.paintDot(paint, p2.x(), p3.y()) # output dot
      # feed back path
      paint.drawLine(QtCore.QPoint(p2.x(), p3.y()), p2) # feedback vertical 
      if (mymw.ui.reg_vel_LeadFwd.isChecked()):    
        paint.drawLine(p2, QtCore.QPoint(p1.x(), p2.y())) # feedback vertical 
      else:
        paint.drawLine(p2, QtCore.QPoint(mymw.ui.frame_vel_lead.x() + mymw.ui.frame_vel_lead.width(), p2.y())) # feedback vertical 
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_vel_lead.x(), p2.y()), 
                       QtCore.QPoint(p1.x(), p2.y())) # feedback hirizontal to lead
        self.paintArrow(paint, mymw.ui.frame_vel_lead.x() + mymw.ui.frame_vel_lead.width(), p2.y(), 2) # to lead in feed back
      # from sum circle down to horizontal line
      paint.drawLine(QtCore.QPoint(p1.x(), p1.y() + 14), 
                     QtCore.QPoint(p1.x(), p2.y()))     # sum south feedback
      #        
      # from limit to motor to gear
      self.paintArrow(paint, mymw.ui.frame_vel_limit.x(), p3.y(), 0) # to motor
      paint.drawLine(QtCore.QPoint(mymw.ui.frame_vel_limit.x() + mymw.ui.frame_vel_limit.width(), p3.y()), 
                     QtCore.QPoint(mymw.ui.frame_vel_motor.x(), p3.y())) # from motor to gear
      self.paintArrow(paint, mymw.ui.frame_vel_motor.x(), p3.y(), 0) # to motor
      paint.drawLine(QtCore.QPoint(mymw.ui.frame_vel_motor.x() + mymw.ui.frame_vel_motor.width(), p3.y()), 
                     QtCore.QPoint(mymw.ui.frame_vel_gear.x(), p3.y())) # from motor to gear
      self.paintArrow(paint, mymw.ui.frame_vel_gear.x(), p3.y(), 0) # to gear
      #paint.drawLine(p3, QtCore.QPoint(mymw.ui.frame_vel_motor.x(), p3.y())) # from dot to limit
      # corner point from step to reference
      p5 = QtCore.QPoint(mymw.ui.frame_vel_step.x() + mymw.ui.frame_vel_step.width()/2, 
                         mymw.ui.frame_vel_step.y() + mymw.ui.frame_vel_step.height())
      p6 = QtCore.QPoint(mymw.ui.frame_vel_step.x() + mymw.ui.frame_vel_step.width(), 
                         mymw.ui.frame_vel_step.y() + mymw.ui.frame_vel_step.height() - 50)
      paint.drawLine(QtCore.QPoint(p5.x(), p1.y()), QtCore.QPoint(p1.x() - 14, p1.y())) # from step to ref
      # to limit
      if (mymw.ui.reg_vel_use.isChecked()):
        # from regulator frame to lead or limit
        if (mymw.ui.reg_vel_LeadFwd.isChecked()):
          # point out of controller frame
          p7 = QtCore.QPoint(mymw.ui.frame_vel_reg.x() + mymw.ui.frame_vel_reg.width(),
                             mymw.ui.frame_vel_lead_fwd.y() + mymw.ui.frame_vel_lead_fwd.height()/2)
          paint.drawLine(p7, QtCore.QPoint(mymw.ui.frame_vel_lead_fwd.x(), p7.y())) # from reg to dot
          self.paintArrow(paint, mymw.ui.frame_vel_lead_fwd.x(), p7.y(), 0) # to gear
          paint.drawLine(QtCore.QPoint(mymw.ui.frame_vel_lead_fwd.x() + mymw.ui.frame_vel_lead_fwd.width(), p7.y()),
                         QtCore.QPoint(p3.x(), p7.y())) # from reg to dot
          paint.drawLine(QtCore.QPoint(p3.x(), p7.y()),
                         QtCore.QPoint(p3.x(), p3.y())) # from reg to dot
          pass
        else:
          paint.drawLine(QtCore.QPoint(mymw.ui.frame_vel_reg.x() + mymw.ui.frame_vel_reg.width(), p3.y()), p3) # from reg to dot
        paint.drawLine(p6, QtCore.QPoint(p1.x(), p6.y())) # from step to ref
        paint.drawLine(QtCore.QPoint(p1.x(), p6.y()), QtCore.QPoint(p1.x(), p1.y() - 14)) # from step to ref
        self.paintArrow(paint, p1.x(), p1.y() - 14, 3)
        self.paintPlus(paint, p1.x(), p1.y() - 8)
      else:
        # from step to limit
        # point above ctrl box - right
        p4 = QtCore.QPoint(p3.x(), mymw.ui.frame_vel_reg.y() - 20)
        paint.drawLine(p4, p3) # from step to dot
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_vel_step.x() + mymw.ui.frame_vel_step.width(), p4.y()), p4) # from step to dot
      paint.end()
# class end

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class PaintTurnSpace(PaintSupport):
  # """My widget for drawing smth"""
  #def __init__(self):
      #super(PaintSpace, self).__init__()
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.black)
      # sum turn and vel
      p1 = QtCore.QPoint(295,147)  # input sum left
      p2 = QtCore.QPoint(295,267)  # input sum right
      p3 = QtCore.QPoint(44,307)   # controller input
      p4 = QtCore.QPoint(410,210)   # turn diff
      paint.drawEllipse(p1, 14, 14)
      self.paintPlus(paint, p1.x() - 8, p1.y())
      self.paintMinus(paint, p1.x(), p1.y() + 8)
      self.paintArrow(paint, p1.x() - 14, p1.y(), 0)
      self.paintArrow(paint, p1.x(), p1.y() + 14, 1)
      paint.drawEllipse(p2, 14, 14)
      self.paintPlus(paint, p2.x() - 8, p2.y())
      self.paintPlus(paint, p2.x(), p2.y() + 8)
      self.paintArrow(paint, p2.x() - 14, p2.y(), 0)
      self.paintArrow(paint, p2.x(), p2.y() + 14, 1)
      paint.drawEllipse(p3, 14, 14)
      self.paintPlus(paint, p3.x() - 8, p3.y())
      self.paintMinus(paint, p3.x(), p3.y() + 8)
      self.paintArrow(paint, p3.x() - 14, p3.y(), 0)
      self.paintArrow(paint, p3.x(), p3.y() + 14, 1)
      paint.drawEllipse(p4, 14, 14)
      self.paintMinus(paint, p4.x(), p4.y() - 8)
      self.paintPlus(paint, p4.x(), p4.y() + 8)
      self.paintArrow(paint, p4.x(), p4.y() - 14, 4)
      self.paintArrow(paint, p4.x(), p4.y() + 14, 1)
      # p1,p2 to motors and sum
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), 
                     QtCore.QPoint(mymw.ui.frame_left_motor.x(), p1.y())) # to east to left motor
      self.paintArrow(paint, mymw.ui.frame_left_motor.x(), p1.y(), 0)
      paint.drawLine(QtCore.QPoint(390, p1.y()), QtCore.QPoint(p4.x(), p1.y())) # left motor to east 
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p2.y()), 
                     QtCore.QPoint(mymw.ui.frame_right_motor.x(), p2.y())) # to east to right motor
      self.paintArrow(paint, mymw.ui.frame_right_motor.x(), p2.y(), 0)
      paint.drawLine(QtCore.QPoint(390, p2.y()), QtCore.QPoint(p4.x(), p2.y())) # right motor to east
      paint.drawLine(QtCore.QPoint(p4.x(), p1.y()), QtCore.QPoint(p4.x(), p4.y() - 14)) # down to turn sum
      paint.drawLine(QtCore.QPoint(p4.x(), p4.y() + 14), QtCore.QPoint(p4.x(), p2.y())) # up to turn sum
      # to wheel radius and B
      paint.drawLine(QtCore.QPoint(p4.x() + 14, p4.y()), 
                     QtCore.QPoint(mymw.ui.frame_turn_wr_b.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_turn_wr_b.x(), p4.y(), 0)
      # to 1/s
      p5 = QtCore.QPoint(mymw.ui.frame_turn_wr_b.x() + mymw.ui.frame_turn_wr_b.width(), p4.y())
      paint.drawLine(p5, QtCore.QPoint(mymw.ui.frame_turn_s.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_turn_s.x(), p4.y(), 0)
      paint.drawLine(QtCore.QPoint(p5.x() + 7, p4.y() + 7), QtCore.QPoint(p5.x() + 7, mymw.ui.label_turn_turnrate.y()))
      # to out (heading)
      p6 = QtCore.QPoint(mymw.ui.frame_turn_s.x() + mymw.ui.frame_turn_s.width(), p4.y())
      paint.drawLine(p6, QtCore.QPoint(p6.x() + 20, p4.y()))
      # down to feedback
      if (mymw.ui.reg_turn_LeadFwd.isChecked()):
        p7 = QtCore.QPoint(p6.x() + 10, 349)
      else:
        p7 = QtCore.QPoint(p6.x() + 10, 399)
      self.paintDot(paint, p7.x(), p4.y())
      paint.drawLine(QtCore.QPoint(p6.x() + 10, 210), p7)
      if (mymw.ui.reg_turn_LeadFwd.isChecked()):
        paint.drawLine(p7, QtCore.QPoint(p3.x(), p7.y()))
      else:
        paint.drawLine(p7, QtCore.QPoint(mymw.ui.frame_turn_reg_lead.x() + mymw.ui.frame_turn_reg_lead.width(), p7.y()))
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_turn_reg_lead.x(), p7.y()), QtCore.QPoint(p3.x(), p7.y()))
      paint.drawLine(QtCore.QPoint(p3.x(), p7.y()), QtCore.QPoint(p3.x(), p3.y() + 14))
      paint.drawLine(QtCore.QPoint(p3.x() + 14, p3.y()), 
                     QtCore.QPoint(mymw.ui.frame_turn_reg.x(), p3.y())) # to regulator
      self.paintArrow(paint, mymw.ui.frame_turn_reg.x(), p3.y(), 0)
      # ref input
      paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 14, p3.y()))
      # from controller to summs
      if (mymw.ui.reg_turn_use.isChecked()):
        p8 = QtCore.QPoint(250, p3.y())
        paint.drawLine(QtCore.QPoint(234, p3.y()), p8)
        self.paintDot(paint, p8.x(), p8.y())
        paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 24, 
                             mymw.ui.frame_turn_step.y() + mymw.ui.frame_turn_step.height()))
      else:
        # from step to summs
        p8 = QtCore.QPoint(250, p4.y()) # dot point
        p9 = QtCore.QPoint(mymw.ui.frame_turn_step.x() + mymw.ui.frame_turn_step.width(), 
                           mymw.ui.frame_turn_step.y() + mymw.ui.frame_turn_step.height() - 20)
        paint.drawLine(p8, QtCore.QPoint(p8.x(), p9.y()))
        paint.drawLine(QtCore.QPoint(p8.x(), p9.y()), p9)
        self.paintDot(paint, p8.x(), p8.y())
      # turn u value
      paint.drawLine(QtCore.QPoint(250, p3.y()), QtCore.QPoint(p2.x(), p3.y()))
      paint.drawLine(QtCore.QPoint(250, p3.y()), QtCore.QPoint(250, p4.y()))
      paint.drawLine(QtCore.QPoint(p2.x(), p3.y()), QtCore.QPoint(p2.x(), p2.y() + 14))
      paint.drawLine(QtCore.QPoint(250, p4.y()), QtCore.QPoint(p1.x(), p4.y()))
      paint.drawLine(QtCore.QPoint(p1.x(), p4.y()), QtCore.QPoint(p1.x(), p1.y() + 14))
      # from velocity to summs
      #paint.drawLine(QtCore.QPoint(180, 160), QtCore.QPoint(p1.x() - 22, 160))
      paint.drawLine(QtCore.QPoint(p1.x() - 27, p1.y()), QtCore.QPoint(p1.x() - 14, p1.y()))
      paint.drawLine(QtCore.QPoint(p1.x() - 27, mymw.ui.frame_turn_vel.y() + mymw.ui.frame_turn_vel.height() ), 
                     QtCore.QPoint(p1.x() - 27, p2.y()))
      paint.drawLine(QtCore.QPoint(p1.x() - 27, p2.y()), QtCore.QPoint(p2.x() - 14, p2.y()))
      # fraction lines
      paint.setPen(QtCore.Qt.black)
      paint.drawLine(QtCore.QPoint(450, p4.y()), QtCore.QPoint(470, p4.y()))
      paint.drawLine(QtCore.QPoint(510, p4.y()), QtCore.QPoint(530, p4.y()))
      #
      paint.end()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class PaintWallSpace(PaintSupport):
  # paint support lines in wallfollow page
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      #paint.setPen(QtCore.Qt.black)
      pen = QtGui.QPen(QtCore.Qt.black, 1, QtCore.Qt.SolidLine)
      #paint.setPen(QtCore.Qt.black)
      paint.setPen(pen)

      # sum turn and vel
      p1 = QtCore.QPoint(mymw.ui.frame_wall_vel.x() + mymw.ui.frame_wall_vel.width() / 2,
                         mymw.ui.frame_left_motor_4.y() + mymw.ui.frame_left_motor_4.height() / 2)  # input sum left
      p2 = QtCore.QPoint(p1.x(), mymw.ui.frame_right_motor_4.y() + mymw.ui.frame_right_motor_4.height() / 2)  # input sum right
      p3 = QtCore.QPoint(44,307)   # controller input
      # output of motors summation point
      p40 = QtCore.QPoint(mymw.ui.frame_right_motor_4.x() + mymw.ui.frame_right_motor_4.width(), p1.y())
      p4 = QtCore.QPoint(p40.x() + 20, mymw.ui.frame_wall_wr_b.y() + mymw.ui.frame_wall_wr_b.height() / 2)   # turn diff
      
      paint.drawEllipse(p1, 14, 14) # left motor sum
      self.paintPlus(paint, p1.x() - 8, p1.y())
      self.paintArrow(paint, p1.x() - 14, p1.y(), 0)
      self.paintArrow(paint, p1.x(), p1.y() + 14, 1)
      paint.drawEllipse(p2, 14, 14) # right motor sum
      self.paintPlus(paint, p2.x() - 8, p2.y())
      self.paintArrow(paint, p2.x() - 14, p2.y(), 0)
      self.paintArrow(paint, p2.x(), p2.y() + 14, 1)
      # set signs in motor control input summation circle
      if mymw.ui.wall_follow_left.isChecked():
        #if mymw.ui.wall_control_sign.isChecked():
          #self.paintPlus(paint, p1.x(), p1.y() + 8)
          #self.paintMinus(paint, p2.x(), p2.y() + 8)
        #else:
        self.paintMinus(paint, p1.x(), p1.y() + 8)
        self.paintPlus(paint, p2.x(), p2.y() + 8)
        self.paintMinus(paint, p4.x(), p4.y() - 8)
        self.paintPlus(paint, p4.x(), p4.y() + 8)
        mymw.ui.label_260.setText("B")
        mymw.ui.label_256.setText("Heading")
        mymw.ui.label_wall_turnrate.setText("turn rate")
        mymw.ui.label_254.setText("r/s")
      else:
        #if mymw.ui.wall_control_sign.isChecked():
          #self.paintMinus(paint, p1.x(), p1.y() + 8)
          #self.paintMinus(paint, p2.x(), p2.y() + 8)
        #else:
        self.paintPlus(paint, p1.x(), p1.y() + 8)
        self.paintPlus(paint, p2.x(), p2.y() + 8)
        self.paintPlus(paint, p4.x(), p4.y() - 8)
        self.paintPlus(paint, p4.x(), p4.y() + 8)
        mymw.ui.label_260.setText("2")
        mymw.ui.label_256.setText("Position")
        mymw.ui.label_wall_turnrate.setText("velocity")
        mymw.ui.label_254.setText("m/s")
      paint.drawEllipse(p3, 14, 14) # ref input sum
      self.paintPlus(paint, p3.x() - 8, p3.y())
      self.paintMinus(paint, p3.x(), p3.y() + 8)
      self.paintArrow(paint, p3.x() - 14, p3.y(), 0)
      self.paintArrow(paint, p3.x(), p3.y() + 14, 1)
      paint.drawEllipse(p4, 14, 14) # difference of motor velocity
      self.paintArrow(paint, p4.x(), p4.y() - 14, 4)
      self.paintArrow(paint, p4.x(), p4.y() + 14, 1)
      # p1,p2 to motors and sum
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), 
                    QtCore.QPoint(mymw.ui.frame_left_motor_4.x(), p1.y())) # to east to left motor
      self.paintArrow(paint, mymw.ui.frame_left_motor_4.x(), p1.y(), 0)
      paint.drawLine(QtCore.QPoint(p40.x(), p1.y()), QtCore.QPoint(p4.x(), p1.y())) # left motor to east 
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p2.y()), 
                    QtCore.QPoint(mymw.ui.frame_right_motor_4.x(), p2.y())) # to east to right motor
      self.paintArrow(paint, mymw.ui.frame_right_motor_4.x(), p2.y(), 0)
      paint.drawLine(QtCore.QPoint(p40.x(), p2.y()), QtCore.QPoint(p4.x(), p2.y())) # right motor to east
      paint.drawLine(QtCore.QPoint(p4.x(), p1.y()), QtCore.QPoint(p4.x(), p4.y() - 14)) # down to turn sum
      paint.drawLine(QtCore.QPoint(p4.x(), p4.y() + 14), QtCore.QPoint(p4.x(), p2.y())) # up to turn sum
      # to wheel radius and B
      paint.drawLine(QtCore.QPoint(p4.x() + 14, p4.y()), 
                    QtCore.QPoint(mymw.ui.frame_wall_wr_b.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_wall_wr_b.x(), p4.y(), 0)
      # to 1/s
      p5 = QtCore.QPoint(mymw.ui.frame_wall_wr_b.x() + mymw.ui.frame_wall_wr_b.width(), p4.y()) # out of wr_b
      paint.drawLine(p5, QtCore.QPoint(mymw.ui.frame_wall_s.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_wall_s.x(), p4.y(), 0)
      # line down to velocity label
      paint.drawLine(QtCore.QPoint(p5.x() + 7, p4.y() + 7), QtCore.QPoint(p5.x() + 7, mymw.ui.label_wall_turnrate.y()))
      # to out (heading)
      p6 = QtCore.QPoint(mymw.ui.frame_wall_s.x() + mymw.ui.frame_wall_s.width(), p4.y()) # just out of 1/s
      paint.drawLine(p6, QtCore.QPoint(p6.x() + 40, p4.y()))
      # down to feedback
      pen.setStyle(QtCore.Qt.DashLine)
      paint.setPen(pen)
      paint.drawLine(QtCore.QPoint(p6.x() + 20, p4.y()), QtCore.QPoint(p6.x() + 20, mymw.ui.frame_wall_sensor.y())) 
      pen.setStyle(QtCore.Qt.SolidLine)
      paint.setPen(pen)
      if (mymw.ui.reg_wall_LeadFwd.isChecked()):
        p7 = QtCore.QPoint(mymw.ui.frame_wall_sensor.x(), mymw.ui.frame_wall_sensor.y() + 20) # left of sensor
        paint.drawLine(p7, QtCore.QPoint(p3.x(), p7.y())) # horizontal line feedback - no lead
      else:
        p7 = QtCore.QPoint(mymw.ui.frame_wall_sensor.x(), mymw.ui.frame_wall_reg_lead.y() + 20) # left of sensor
        paint.drawLine(p7, QtCore.QPoint(mymw.ui.frame_wall_reg_lead.x() + mymw.ui.frame_wall_reg_lead.width(), p7.y()))
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_wall_reg_lead.x(), p7.y()), QtCore.QPoint(p3.x(), p7.y()))
      #self.paintDot(paint, p7.x(), p4.y())
      #paint.drawLine(QtCore.QPoint(p6.x() + 10, 210), p7)
      paint.drawLine(QtCore.QPoint(p3.x(), p7.y()), QtCore.QPoint(p3.x(), p3.y() + 14)) # feedback up to refinput circle
      paint.drawLine(QtCore.QPoint(p3.x() + 14, p3.y()), 
                     QtCore.QPoint(mymw.ui.frame_wall_reg.x(), p3.y())) # from ref input circle to regulator
      self.paintArrow(paint, mymw.ui.frame_wall_reg.x(), p3.y(), 0)
      # ref input
      paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 14, p3.y()))
      # from controller to summs
      # out of regulator - low
      p8 = QtCore.QPoint(mymw.ui.frame_wall_reg.x() + mymw.ui.frame_wall_reg.width(), 
                         mymw.ui.frame_wall_reg.y() + mymw.ui.frame_wall_reg.height() - 30)
      paint.drawLine(QtCore.QPoint(p1.x(), p8.y()), p8)
      self.paintDot(paint, p8.x() + 20, p8.y())
      # regulator input from step
      paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 24, 
                            mymw.ui.frame_wall_step.y() + mymw.ui.frame_wall_step.height()))
      # out of regulator to motor sums
      #paint.drawLine(QtCore.QPoint(p8.x() + 20, p8.y()), QtCore.QPoint(p2.x(), p3.y()))
      paint.drawLine(QtCore.QPoint(p8.x() + 20, p8.y()), QtCore.QPoint(p8.x() + 20, p4.y())) # lodret til midt
      paint.drawLine(QtCore.QPoint(p2.x(), p8.y()), QtCore.QPoint(p2.x(), p2.y() + 14)) # lodret til right sum
      paint.drawLine(QtCore.QPoint(p8.x() + 20, p4.y()), QtCore.QPoint(p1.x(), p4.y())) # horizontal between sums
      paint.drawLine(QtCore.QPoint(p1.x(), p4.y()), QtCore.QPoint(p1.x(), p1.y() + 14)) # up to left motor sum
      # from velocity to summs
      paint.drawLine(QtCore.QPoint(p1.x() - 27, p1.y()), QtCore.QPoint(p1.x() - 14, p1.y()))
      if mymw.ui.wall_follow_left.isChecked():
        # base velocity vertical line valid only while following wall
        paint.drawLine(QtCore.QPoint(p1.x() - 27, mymw.ui.frame_wall_vel.y() + mymw.ui.frame_wall_vel.height() ), 
                     QtCore.QPoint(p1.x() - 27, p2.y()))
      paint.drawLine(QtCore.QPoint(p1.x() - 27, p2.y()), QtCore.QPoint(p2.x() - 14, p2.y()))
      # fraction lines
      paint.setPen(QtCore.Qt.black)
      p91 = QtCore.QPoint(mymw.ui.frame_wall_wr_b.x() + 10, p4.y())
      p92 = QtCore.QPoint(mymw.ui.frame_wall_s.x() + 10, p4.y())
      paint.drawLine(p91, QtCore.QPoint(p91.x() + 20, p4.y()))
      paint.drawLine(p92, QtCore.QPoint(p92.x() + 20, p4.y()))
      #
      paint.end()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class PaintPositionSpace(PaintSupport):
  # """My widget for drawing smth"""
  #def __init__(self):
      #super(PaintSpace, self).__init__()
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.black)
      # sum turn and vel
      p1 = QtCore.QPoint(mymw.ui.frame_left_motor_3.x(), mymw.ui.frame_left_motor_3.y()  + mymw.ui.frame_left_motor_3.height()/2)  # input sum left
      p2 = QtCore.QPoint(mymw.ui.frame_right_motor_3.x(), mymw.ui.frame_right_motor_3.y()  + mymw.ui.frame_right_motor_3.height()/2)  # input sum left
      p3 = QtCore.QPoint(44,307)   # controller input
      p4 = QtCore.QPoint(410,210)   # turn diff
      #paint.drawEllipse(p1, 14, 14)
      #self.paintPlus(paint, p1.x() - 8, p1.y())
      #self.paintMinus(paint, p1.x(), p1.y() + 8)
      #self.paintArrow(paint, p1.x() - 14, p1.y(), 0)
      #self.paintArrow(paint, p1.x(), p1.y() + 14, 1)
      #paint.drawEllipse(p2, 14, 14)
      #self.paintPlus(paint, p2.x() - 8, p2.y())
      #self.paintPlus(paint, p2.x(), p2.y() + 8)
      #self.paintArrow(paint, p2.x() - 14, p2.y(), 0)
      #self.paintArrow(paint, p2.x(), p2.y() + 14, 1)
      paint.drawEllipse(p3, 14, 14)
      self.paintPlus(paint, p3.x() - 8, p3.y())
      self.paintMinus(paint, p3.x(), p3.y() + 8)
      self.paintArrow(paint, p3.x() - 14, p3.y(), 0)
      self.paintArrow(paint, p3.x(), p3.y() + 14, 1)
      paint.drawEllipse(p4, 14, 14)
      self.paintPlus(paint, p4.x(), p4.y() - 8)
      self.paintPlus(paint, p4.x(), p4.y() + 8)
      self.paintArrow(paint, p4.x(), p4.y() - 14, 4)
      self.paintArrow(paint, p4.x(), p4.y() + 14, 1)
      # p1,p2 to motors and sum
      paint.drawLine(QtCore.QPoint(p1.x() - 40, p1.y()), p1) # to east to left motor
      self.paintArrow(paint, p1.x(), p1.y(), 0)
      paint.drawLine(QtCore.QPoint(390, p1.y()), QtCore.QPoint(p4.x(), p1.y())) # left motor to east 
      paint.drawLine(QtCore.QPoint(p1.x() - 40, p2.y()), p2) # to east to right motor
      self.paintArrow(paint, p2.x(), p2.y(), 0)
      paint.drawLine(QtCore.QPoint(390, p2.y()), QtCore.QPoint(p4.x(), p2.y())) # right motor to east
      paint.drawLine(QtCore.QPoint(p4.x(), p1.y()), QtCore.QPoint(p4.x(), p4.y() - 14)) # down to turn sum
      paint.drawLine(QtCore.QPoint(p4.x(), p4.y() + 14), QtCore.QPoint(p4.x(), p2.y())) # up to turn sum
      # to wheel radius and B
      paint.drawLine(QtCore.QPoint(p4.x() + 14, p4.y()), 
                     QtCore.QPoint(mymw.ui.frame_pos_vel.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_pos_vel.x(), p4.y(), 0)
      # to 1/s
      p5 = QtCore.QPoint(mymw.ui.frame_pos_vel.x() + mymw.ui.frame_pos_vel.width(), p4.y())
      paint.drawLine(p5, QtCore.QPoint(mymw.ui.frame_pos_s.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_pos_s.x(), p4.y(), 0)
      paint.drawLine(QtCore.QPoint(p5.x() + 7, p4.y() + 7), QtCore.QPoint(p5.x() + 7, mymw.ui.label_turn_turnrate_4.y()))
      # to out (heading)
      p6 = QtCore.QPoint(mymw.ui.frame_pos_s.x() + mymw.ui.frame_pos_s.width(), p4.y())
      paint.drawLine(p6, QtCore.QPoint(p6.x() + 20, p4.y()))
      # down to feedback
      if (mymw.ui.reg_pos_LeadFwd.isChecked()):
        p7 = QtCore.QPoint(p6.x() + 10, 349)
      else:
        p7 = QtCore.QPoint(p6.x() + 10, 399)
      self.paintDot(paint, p7.x(), p4.y())
      paint.drawLine(QtCore.QPoint(p6.x() + 10, 210), p7)
      if (mymw.ui.reg_pos_LeadFwd.isChecked()):
        paint.drawLine(p7, QtCore.QPoint(p3.x(), p7.y()))
      else:
        paint.drawLine(p7, QtCore.QPoint(mymw.ui.frame_pos_reg_lead.x() + mymw.ui.frame_pos_reg_lead.width(), p7.y()))
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_pos_reg_lead.x(), p7.y()), QtCore.QPoint(p3.x(), p7.y()))
      paint.drawLine(QtCore.QPoint(p3.x(), p7.y()), QtCore.QPoint(p3.x(), p3.y() + 14))
      paint.drawLine(QtCore.QPoint(p3.x() + 14, p3.y()), 
                     QtCore.QPoint(mymw.ui.frame_pos_reg.x(), p3.y())) # to regulator
      self.paintArrow(paint, mymw.ui.frame_pos_reg.x(), p3.y(), 0)
      # ref input
      paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 14, p3.y()))
      # from controller to summs
      if (mymw.ui.reg_pos_use.isChecked()):
        p8 = QtCore.QPoint(p1.x() - 40, p2.y()) # lower dot point
        p9 = QtCore.QPoint(p3.x() - 24, mymw.ui.frame_pos_step.y() + mymw.ui.frame_pos_step.height()) # step bottom
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_pos_reg.x() + mymw.ui.frame_pos_reg.width(), p8.y()), p8)
        self.paintDot(paint, p8.x(), p8.y())
        paint.drawLine(p8, QtCore.QPoint(p8.x(), p1.y()))
        # from step to ref input
        paint.drawLine(p9, QtCore.QPoint(p9.x(), p3.y()))
      else:
        # from step to summs
        p8 = QtCore.QPoint(p1.x() - 40, p1.y()) # upper dot point
        p9 = QtCore.QPoint(mymw.ui.frame_pos_step.x() + mymw.ui.frame_pos_step.width(), 
                         mymw.ui.frame_pos_step.y() + mymw.ui.frame_pos_step.height() - 20)
        paint.drawLine(p8, QtCore.QPoint(p8.x(), p9.y()))
        paint.drawLine(QtCore.QPoint(p8.x(), p9.y()), p9)
        self.paintDot(paint, p8.x(), p8.y())
        paint.drawLine(p8, QtCore.QPoint(p8.x(), p2.y()))
      # turn u value
      #paint.drawLine(QtCore.QPoint(250, p3.y()), QtCore.QPoint(p2.x(), p3.y()))
      #paint.drawLine(QtCore.QPoint(250, p3.y()), QtCore.QPoint(250, p4.y()))
      #paint.drawLine(QtCore.QPoint(p2.x(), p3.y()), QtCore.QPoint(p2.x(), p2.y() + 14))
      #paint.drawLine(QtCore.QPoint(250, p4.y()), QtCore.QPoint(p1.x(), p4.y()))
      #paint.drawLine(QtCore.QPoint(p1.x(), p4.y()), QtCore.QPoint(p1.x(), p1.y() + 14))
      # from velocity to summs
      #paint.drawLine(QtCore.QPoint(180, 160), QtCore.QPoint(p1.x() - 22, 160))
      #paint.drawLine(QtCore.QPoint(p1.x() - 27, p1.y()), QtCore.QPoint(p1.x() - 14, p1.y()))
      #paint.drawLine(QtCore.QPoint(p1.x() - 27, mymw.ui.frame_pos_vel.y() + mymw.ui.frame_pos_vel.height() ), 
                     #QtCore.QPoint(p1.x() - 27, p2.y()))
      #paint.drawLine(QtCore.QPoint(p1.x() - 27, p2.y()), QtCore.QPoint(p2.x() - 14, p2.y()))
      # fraction lines
      paint.setPen(QtCore.Qt.black)
      paint.drawLine(QtCore.QPoint(450, p4.y()), QtCore.QPoint(470, p4.y()))
      paint.drawLine(QtCore.QPoint(510, p4.y()), QtCore.QPoint(530, p4.y()))
      #
      paint.end()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class PaintLineSpace(PaintSupport):
  # """My widget for drawing smth"""
  #def __init__(self):
      #super(PaintSpace, self).__init__()
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      pen = QtGui.QPen(QtCore.Qt.black, 1, QtCore.Qt.SolidLine)
      #paint.setPen(QtCore.Qt.black)
      paint.setPen(pen)

      # sum turn and vel
      p1 = QtCore.QPoint(mymw.ui.frame_left_motor_2.x() - 40, 
                         mymw.ui.frame_left_motor_2.y() + mymw.ui.frame_left_motor_2.height() / 2)  # input sum left
      p2 = QtCore.QPoint(p1.x(), mymw.ui.frame_right_motor_2.y() + mymw.ui.frame_right_motor_2.height() / 2)  # input sum right
      p3 = QtCore.QPoint(44, mymw.ui.frame_turn_reg_2.y() + (mymw.ui.frame_turn_reg_2.height() * 4) / 5)   # controller input
      p4 = QtCore.QPoint(mymw.ui.frame_turn_wr_b_2.x() - 40,
                         mymw.ui.frame_turn_wr_b_2.y() + mymw.ui.frame_turn_wr_b_2.height() / 2)   # turn diff summation
      p11 = QtCore.QPoint(mymw.ui.frame_left_motor_2.x() + mymw.ui.frame_left_motor_2.width(), p1.y())  # motor 1 output
      paint.drawEllipse(p1, 14, 14)
      self.paintPlus(paint, p1.x() - 8, p1.y())
      self.paintMinus(paint, p1.x(), p1.y() + 8)
      self.paintArrow(paint, p1.x() - 14, p1.y(), 0)
      self.paintArrow(paint, p1.x(), p1.y() + 14, 1)
      paint.drawEllipse(p2, 14, 14)
      self.paintPlus(paint, p2.x() - 8, p2.y())
      self.paintPlus(paint, p2.x(), p2.y() + 8)
      self.paintArrow(paint, p2.x() - 14, p2.y(), 0)
      self.paintArrow(paint, p2.x(), p2.y() + 14, 1)
      paint.drawEllipse(p3, 14, 14)
      self.paintPlus(paint, p3.x() - 8, p3.y())
      self.paintMinus(paint, p3.x(), p3.y() + 8)
      self.paintArrow(paint, p3.x() - 14, p3.y(), 0)
      self.paintArrow(paint, p3.x(), p3.y() + 14, 1)
      paint.drawEllipse(p4, 14, 14)
      self.paintMinus(paint, p4.x(), p4.y() - 8)
      self.paintPlus(paint, p4.x(), p4.y() + 8)
      self.paintArrow(paint, p4.x(), p4.y() - 14, 4)
      self.paintArrow(paint, p4.x(), p4.y() + 14, 1)
      # p1,p2 to motors and sum
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), 
                     QtCore.QPoint(mymw.ui.frame_left_motor_2.x(), p1.y())) # to east to left motor
      self.paintArrow(paint, mymw.ui.frame_left_motor_2.x(), p1.y(), 0)
      paint.drawLine(p11, QtCore.QPoint(p4.x(), p1.y())) # left motor to east 
      paint.drawLine(QtCore.QPoint(p1.x() + 14, p2.y()), 
                     QtCore.QPoint(mymw.ui.frame_right_motor_2.x(), p2.y())) # to east to right motor
      self.paintArrow(paint, mymw.ui.frame_right_motor_2.x(), p2.y(), 0)
      paint.drawLine(QtCore.QPoint(p11.x(), p2.y()), QtCore.QPoint(p4.x(), p2.y())) # right motor to east
      paint.drawLine(QtCore.QPoint(p4.x(), p1.y()), QtCore.QPoint(p4.x(), p4.y() - 14)) # down to turn sum
      paint.drawLine(QtCore.QPoint(p4.x(), p4.y() + 14), QtCore.QPoint(p4.x(), p2.y())) # up to turn sum
      # to wheel radius and B
      paint.drawLine(QtCore.QPoint(p4.x() + 14, p4.y()), 
                     QtCore.QPoint(mymw.ui.frame_turn_wr_b_2.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_turn_wr_b_2.x(), p4.y(), 0)
      # to 1/s
      p5 = QtCore.QPoint(mymw.ui.frame_turn_wr_b_2.x() + mymw.ui.frame_turn_wr_b_2.width(), p4.y())
      paint.drawLine(p5, QtCore.QPoint(mymw.ui.frame_turn_s_2.x(), p4.y()))
      self.paintArrow(paint, mymw.ui.frame_turn_s_2.x(), p4.y(), 0)
      paint.drawLine(QtCore.QPoint(p5.x() + 7, p4.y() + 7), QtCore.QPoint(p5.x() + 7, mymw.ui.label_turn_turnrate_2.y()))
      # to out (line pos)
      p6 = QtCore.QPoint(mymw.ui.frame_turn_s_2.x() + mymw.ui.frame_turn_s_2.width(), p4.y())
      paint.drawLine(p6, QtCore.QPoint(p6.x() + 20, p4.y()))
      # down to feedback
      if (mymw.ui.reg_turn_LeadFwd_2.isChecked()):
        p7 = QtCore.QPoint(p6.x() + 10, p3.y() + 50)
      else:
        p7 = QtCore.QPoint(p6.x() + 10, mymw.ui.frame_turn_reg_lead_2.y() + 30)
      #self.paintDot(paint, p7.x(), p4.y())
      #
      pen.setStyle(QtCore.Qt.DashLine)
      paint.setPen(pen)
      paint.drawLine(QtCore.QPoint(p6.x() + 10, p4.y()), QtCore.QPoint(p7.x(), mymw.ui.frame_ls_sensor.y()))
      pen.setStyle(QtCore.Qt.SolidLine)
      paint.setPen(pen)
      self.paintArrow(paint, p7.x(), mymw.ui.frame_ls_sensor.y(), 3)
      #
      if (mymw.ui.reg_turn_LeadFwd_2.isChecked()):
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_ls_sensor.x(), p7.y()), QtCore.QPoint(p3.x(), p7.y()))
      else:
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_ls_sensor.x(), p7.y()), QtCore.QPoint(mymw.ui.frame_turn_reg_lead_2.x() + mymw.ui.frame_turn_reg_lead_2.width(), p7.y()))
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_turn_reg_lead_2.x(), p7.y()), QtCore.QPoint(p3.x(), p7.y()))
        self.paintArrow(paint, mymw.ui.frame_turn_reg_lead_2.x() + mymw.ui.frame_turn_reg_lead_2.width(), p7.y(), 2)
      paint.drawLine(QtCore.QPoint(p3.x(), p7.y()), QtCore.QPoint(p3.x(), p3.y() + 14))
      paint.drawLine(QtCore.QPoint(p3.x() + 14, p3.y()), 
                     QtCore.QPoint(mymw.ui.frame_turn_reg_2.x(), p3.y())) # to regulator
      self.paintArrow(paint, mymw.ui.frame_turn_reg_2.x(), p3.y(), 0)
      # ref input
      paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 14, p3.y()))
      # from controller to summs
      #if (mymw.ui.reg_turn_use_2.isChecked()):
      p8 = QtCore.QPoint(mymw.ui.frame_turn_reg_2.x() + mymw.ui.frame_turn_reg_2.width() + 30, p3.y())
      paint.drawLine(QtCore.QPoint(p8.x() - 30, p8.y()), p8)
      self.paintDot(paint, p8.x(), p8.y())
      paint.drawLine(QtCore.QPoint(p3.x() - 24, p3.y()), QtCore.QPoint(p3.x() - 24, 
                            mymw.ui.frame_turn_step_2.y() + mymw.ui.frame_turn_step_2.height()))
      #else:
        ## from step to summs
        #p8 = QtCore.QPoint(mymw.ui.frame_turn_reg_2.x() + mymw.ui.frame_turn_reg_2.width() + 30, p4.y()) # dot point
        #p9 = QtCore.QPoint(mymw.ui.frame_turn_step_2.x() + mymw.ui.frame_turn_step_2.width(), 
                           #mymw.ui.frame_turn_step_2.y() + mymw.ui.frame_turn_step_2.height() - 20)
        #paint.drawLine(p8, QtCore.QPoint(p8.x(), p9.y()))
        #paint.drawLine(QtCore.QPoint(p8.x(), p9.y()), p9)
        #self.paintDot(paint, p8.x(), p8.y())
      # controller out u value
      paint.drawLine(QtCore.QPoint(p8.x(), p3.y()), QtCore.QPoint(p2.x(), p3.y()))
      paint.drawLine(QtCore.QPoint(p8.x(), p3.y()), QtCore.QPoint(p8.x(), p4.y()))
      paint.drawLine(QtCore.QPoint(p2.x(), p3.y()), QtCore.QPoint(p2.x(), p2.y() + 14))
      paint.drawLine(QtCore.QPoint(p8.x(), p4.y()), QtCore.QPoint(p1.x(), p4.y()))
      paint.drawLine(QtCore.QPoint(p1.x(), p4.y()), QtCore.QPoint(p1.x(), p1.y() + 14))
      # from base velocity to summs
      paint.drawLine(QtCore.QPoint(p1.x() - 27, p1.y()), QtCore.QPoint(p1.x() - 14, p1.y()))
      paint.drawLine(QtCore.QPoint(p1.x() - 27, mymw.ui.frame_turn_vel_2.y() + mymw.ui.frame_turn_vel_2.height() ), 
                     QtCore.QPoint(p1.x() - 27, p2.y()))
      paint.drawLine(QtCore.QPoint(p1.x() - 27, p2.y()), QtCore.QPoint(p2.x() - 14, p2.y()))
      # fraction lines
      paint.setPen(QtCore.Qt.black)
      paint.drawLine(QtCore.QPoint(p4.x() + 50, p4.y()), QtCore.QPoint(p4.x() + 70, p4.y()))
      paint.drawLine(QtCore.QPoint(mymw.ui.frame_turn_s_2.x() + 10, p4.y()), QtCore.QPoint(mymw.ui.frame_turn_s_2.x() + 30, p4.y()))
      #
      paint.end()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class PaintBalSpace(PaintSupport):
  # """My widget for drawing smth"""
  #def __init__(self):
      #super(PaintSpace, self).__init__()
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.black)
      # sum turn and vel
      p1 = QtCore.QPoint(70, mymw.ui.frame_bal_ctrl.y() + mymw.ui.frame_bal_ctrl.height()/2)  # input ref
      p2 = QtCore.QPoint(mymw.ui.frame_bal_ctrl.x() + mymw.ui.frame_bal_ctrl.width(), p1.y())  # output balance ctrl
      p3 = QtCore.QPoint(mymw.ui.frame_bal_velctrl.x() + mymw.ui.frame_bal_velctrl.width(), p1.y())  # output mission vel frame
      p4 = QtCore.QPoint(mymw.ui.frame_bal_drivesys.x() + mymw.ui.frame_bal_drivesys.width()/2, 
                         mymw.ui.frame_bal_drivesys.y() + mymw.ui.frame_bal_drivesys.height())  # gyro output of drivesys
      p5  = QtCore.QPoint(mymw.ui.frame_bal_drivesys.x() + mymw.ui.frame_bal_drivesys.width(), p1.y())  # outut of drivesys
      p51 = QtCore.QPoint(mymw.ui.frame_bal_tilt.x() + mymw.ui.frame_bal_tilt.width(), p1.y())  # outut of tilt
      p61 = QtCore.QPoint(p5.x() + 60, p1.y() - mymw.ui.frame_bal_drivesys.height() - 30) # velocity above dot
      
      #hertil
      
      p6 = QtCore.QPoint(p5.x() + 60, p1.y() + mymw.ui.frame_bal_drivesys.height() + 100) # tilt angle output below dot
      p7 = QtCore.QPoint(mymw.ui.frame_bal_ctrl.x() + mymw.ui.frame_bal_ctrl.width()/2, 
                         mymw.ui.frame_bal_ctrl.y() + mymw.ui.frame_bal_ctrl.height()) # bal controller measurent input
      p8 = QtCore.QPoint(mymw.ui.frame_bal_velctrl.x() + mymw.ui.frame_bal_velctrl.width()/2 + 10, 
                         mymw.ui.frame_bal_velctrl.y() + mymw.ui.frame_bal_velctrl.height()) # tilt velocity measurement input
      p9 = QtCore.QPoint(p8.x(), p6.y() - 10) # below gyro input to tilt vel controller
      p11 = QtCore.QPoint(mymw.ui.frame_mvel_reg.x() + 30, mymw.ui.frame_mvel_reg.y() + mymw.ui.frame_mvel_reg.height())
      # input
      #paint.drawEllipse(p1, 14, 14)
      paint.drawLine(QtCore.QPoint(p1.x() - 40, p1.y()), QtCore.QPoint(p1.x() - 14, p1.y()))
      paint.drawEllipse(p1, 14, 14)
      self.paintPlus(paint, p1.x() - 8, p1.y())
      self.paintPlus(paint, p1.x(), p1.y() + 8)
      self.paintArrow(paint, p1.x() - 14, p1.y(), 0)
      self.paintArrow(paint, p1.x(), p1.y() + 14, 1)
      # line from step about_box
      paint.drawLine(QtCore.QPoint(p1.x(), mymw.ui.frame_bal_step.y()), QtCore.QPoint(p1.x(), p1.y() + 14))     
      # output of drive system and tilt detect
      #drivesystem output
      paint.drawLine(p5, QtCore.QPoint(p6.x() + 30, p1.y()))
      #self.paintArrow(paint, mymw.ui.frame_bal_tilt.x(), p1.y(), 0)
      self.paintDot(paint, p6.x(), p1.y())
      # tilt detect output
      
      # mission velocity block lines
      if (mymw.ui.reg_balvel_use.isChecked()):
        mymw.ui.frame_bal_velctrl.setVisible(True)
        # from params to block
        paint.drawLine(p11, QtCore.QPoint(mymw.ui.frame_bal_velctrl.x() + 30, mymw.ui.frame_bal_velctrl.y()))
        # from ref to speed
        paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), QtCore.QPoint(mymw.ui.frame_bal_velctrl.x(), p1.y()))
        self.paintArrow(paint, mymw.ui.frame_bal_velctrl.x(), p1.y(), 0)
        if (mymw.ui.reg_bal_use.isChecked()):
          # from mission vel to balance box
          paint.drawLine(p3, QtCore.QPoint(mymw.ui.frame_bal_ctrl.x(), p1.y()))
          self.paintArrow(paint, mymw.ui.frame_bal_ctrl.x(), p1.y(), 0)
          mymw.ui.label_bal_rad_sec.setVisible(True)
        else:
          # from bal direct to drive
          paint.drawLine(p3, QtCore.QPoint(mymw.ui.frame_bal_drivesys.x(), p1.y()))
          mymw.ui.label_bal_rad_sec.setVisible(False)
        # tilt velocity feedback
        self.paintDot(paint, p6.x(), p1.y())
        paint.drawLine(QtCore.QPoint(p6.x(), p1.y()), p6)
        paint.drawLine(p6, QtCore.QPoint(p8.x(), p6.y()))
        paint.drawLine(QtCore.QPoint(p8.x(), p6.y()), p8)
        self.paintArrow(paint, p8.x(), p8.y(), 1)
        pass
      else:
        mymw.ui.frame_bal_velctrl.setVisible(False)
        mymw.ui.label_bal_rad_sec.setVisible(False)
        if (not mymw.ui.reg_bal_use.isChecked()):
          paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), QtCore.QPoint(mymw.ui.frame_bal_drivesys.x(), p1.y()))
        self.paintArrow(paint, mymw.ui.frame_bal_drivesys.x(), p1.y(), 0)
        pass
      if (mymw.ui.reg_bal_use.isChecked()):
        mymw.ui.frame_bal_ctrl.setVisible(True)
        paint.drawLine(p2, QtCore.QPoint(mymw.ui.frame_bal_drivesys.x(), p1.y()))
        # line to control parameters
        paint.drawLine(QtCore.QPoint(mymw.ui.frame_bal_reg.x() + 20, mymw.ui.frame_bal_reg.y()+mymw.ui.frame_bal_reg.height()), 
                       QtCore.QPoint(mymw.ui.frame_bal_ctrl.x() + 40, mymw.ui.frame_bal_ctrl.y()))
        # make feedback visible
        mymw.ui.frame_bal_tilt.setVisible(True)
        mymw.ui.label_111.setVisible(True)
        mymw.ui.label_112.setVisible(True)
        # feed back lines
        p200 = QtCore.QPoint(mymw.ui.frame_bal_tilt.x(), mymw.ui.label_111.y())
        p201 = QtCore.QPoint(p200.x(), mymw.ui.label_112.y())
        paint.drawLine(QtCore.QPoint(p200.x() + 20, p4.y()), QtCore.QPoint(p200.x() + 20, mymw.ui.frame_bal_tilt.y()))
        paint.drawLine(p200, QtCore.QPoint(p7.x() + 20, p200.y()))
        paint.drawLine(QtCore.QPoint(p7.x() + 20, p200.y()), QtCore.QPoint(p7.x() + 20, p7.y()))
        self.paintArrow(paint, p7.x() + 20, p7.y(), 1)
        paint.drawLine(p201, QtCore.QPoint(p7.x() - 20, p201.y()))
        paint.drawLine(QtCore.QPoint(p7.x() - 20, p201.y()), QtCore.QPoint(p7.x() - 20, p7.y()))
        self.paintArrow(paint, p7.x() - 20, p7.y(), 1)
        if (not mymw.ui.reg_balvel_use.isChecked()):
          # from ref to speed
          paint.drawLine(QtCore.QPoint(p1.x() + 14, p1.y()), QtCore.QPoint(mymw.ui.frame_bal_ctrl.x(), p1.y()))
          self.paintArrow(paint, mymw.ui.frame_bal_ctrl.x(), p1.y(), 0)
      else:
        mymw.ui.frame_bal_ctrl.setVisible(False)        
        mymw.ui.frame_bal_tilt.setVisible(False)
        mymw.ui.label_111.setVisible(False)
        mymw.ui.label_112.setVisible(False)
      self.paintArrow(paint, mymw.ui.frame_bal_drivesys.x(), p1.y(), 0) 
      mymw.ui.label_bal_out_unit.setVisible(True)
        #
      paint.end()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

class paintSS_Space(PaintSupport):
  # """My widget for drawing smth"""
  #def __init__(self):
      #super(PaintSpace, self).__init__()
  def paintEvent(self, QPaintEvent):
      """Reimpltmented drawing method of my widget"""
      paint = QtGui.QPainter()
      paint.begin(self)
      paint.setPen(QtCore.Qt.black)
      # sum turn and vel
      p4 = QtCore.QPoint(mymw.ui.frame_ss_step.x() + mymw.ui.frame_ss_step.width()/2, 
                         mymw.ui.frame_ss_step.y() + mymw.ui.frame_ss_step.height() + 60)  # ref input sum
      # input sum
      paint.drawEllipse(p4, 14, 14)
      self.paintPlus(paint, p4.x() - 8, p4.y())
      self.paintArrow(paint, p4.x() - 14, p4.y(), 0)
      paint.end()

# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////
# ////////////////////////////////////////////////////////

  
robot = URobot()
   

#class UMainWindow(QtGui.QMainWindow):
class UMainWindow(QtGui.QMainWindow):
  def __init__(self, parent=None):
    QtGui.QWidget.__init__(self, parent)
    #QDialog.__init__(self, parent)
    self.ui = Ui_regbot()
    print("starting ...")
    self.ui.setupUi(self)
    self.timer = QtCore.QTimer(self)
    #self.timer = QTimer(self)
    self.timer.timeout.connect(robot.timerUpdate)
    self.ui.statusbar.showMessage("Robot client starting - not connected")
    # add usb connection frame
    self.usb_paint_space = VelocityPaintSpace(self.ui.frame_usb_connect)
    self.usb_paint_space.setGeometry(QtCore.QRect(0, 0, 300, 100))
    self.usb_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.usb_paint_space.lower()
    # paint wifi connection frame
    self.wifi_paint_space = VelocityPaintSpace(self.ui.frame_wifi_connect)
    self.wifi_paint_space.setGeometry(QtCore.QRect(0, 0, 300, 100))
    self.wifi_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.wifi_paint_space.lower()
    # velocity tab
    self.vel_paint_space = VelocityPaintSpace(self.ui.reg_vel_frame)
    self.vel_paint_space.setGeometry(QtCore.QRect(0, 0, 680, 380))
    self.vel_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.vel_paint_space.lower()
    # position tab
    self.pos_paint_space = PaintPositionSpace(self.ui.reg_pos_frame)
    self.pos_paint_space.setGeometry(QtCore.QRect(0, 0, 680, 430))
    self.pos_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.pos_paint_space.lower()
    # turn tab
    self.turn_paint_space = PaintTurnSpace(self.ui.turn_regul_frame)
    self.turn_paint_space.setGeometry(QtCore.QRect(0, 0, 580, 430))
    self.turn_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.turn_paint_space.lower()
    # balance tab 
    self.bal_paint_space = PaintBalSpace(self.ui.bal_regul_frame)
    self.bal_paint_space.setGeometry(QtCore.QRect(0, 0, 580, 610))
    self.bal_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.bal_paint_space.lower()
    # balance velocity tab 
    self.bal_vel_paint_space = paintSS_Space(self.ui.bal_ss_frame)
    self.bal_vel_paint_space.setGeometry(QtCore.QRect(0, 0, 580, 510))
    self.bal_vel_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.bal_vel_paint_space.lower()
    # line-drive tab
    self.ls_paint_space = PaintLineSpace(self.ui.ls_control_frame)
    self.ls_paint_space.setGeometry(QtCore.QRect(0, 0, 680, 520))
    self.ls_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.ls_paint_space.lower()
    # wall follow
    self.wall_paint_space = PaintWallSpace(self.ui.wall_control_frame)
    self.wall_paint_space.setGeometry(QtCore.QRect(0, 0, 680, 520))
    self.wall_paint_space.setFocusPolicy(QtCore.Qt.NoFocus)
    self.wall_paint_space.lower()
    # self.turn_paint_space.setFocusProxy(self.ui.turn_regul_frame)
    # paint background lightGray
    p = self.ui.centralwidget.palette()
    p.setColor(self.ui.centralwidget.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.centralwidget.setPalette(p)
    self.ui.centralwidget.setAutoFillBackground(True)
    #
    p = self.ui.bal_regul_frame.palette()
    p.setColor(self.ui.bal_regul_frame.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.bal_regul_frame.setPalette(p)
    self.ui.bal_regul_frame.setAutoFillBackground(True)
    #
    p = self.ui.turn_regul_frame.palette()
    p.setColor(self.ui.turn_regul_frame.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.turn_regul_frame.setPalette(p)
    self.ui.turn_regul_frame.setAutoFillBackground(True)
    #
    p = self.ui.speed_ctrl.palette()
    p.setColor(self.ui.speed_ctrl.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.speed_ctrl.setPalette(p)
    self.ui.speed_ctrl.setAutoFillBackground(True)
    #
    p = self.ui.main.palette()
    p.setColor(self.ui.main.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.main.setPalette(p)
    self.ui.main.setAutoFillBackground(True)
    #
    p = self.ui.tab.palette()
    p.setColor(self.ui.tab.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab.setPalette(p)
    self.ui.tab.setAutoFillBackground(True)
    #
    p = self.ui.tab_2.palette()
    p.setColor(self.ui.tab_2.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_2.setPalette(p)
    self.ui.tab_2.setAutoFillBackground(True)
    #
    p = self.ui.tab_3.palette()
    p.setColor(self.ui.tab_3.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_3.setPalette(p)
    self.ui.tab_3.setAutoFillBackground(True)
    #
    p = self.ui.tab_4.palette()
    p.setColor(self.ui.tab_4.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_4.setPalette(p)
    self.ui.tab_4.setAutoFillBackground(True)
    #
    p = self.ui.tab_5.palette()
    p.setColor(self.ui.tab_5.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_5.setPalette(p)
    self.ui.tab_5.setAutoFillBackground(True)
    #
    p = self.ui.tab_6.palette()
    p.setColor(self.ui.tab_6.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_6.setPalette(p)
    self.ui.tab_6.setAutoFillBackground(True)
    #
    p = self.ui.tab_7.palette()
    p.setColor(self.ui.tab_7.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_7.setPalette(p)
    self.ui.tab_7.setAutoFillBackground(True)
    #
    p = self.ui.tab_8.palette()
    p.setColor(self.ui.tab_8.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_8.setPalette(p)
    self.ui.tab_8.setAutoFillBackground(True)
    #
    p = self.ui.tab_9.palette()
    p.setColor(self.ui.tab_9.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_9.setPalette(p)
    self.ui.tab_9.setAutoFillBackground(True)
    #
    p = self.ui.tab_10.palette()
    p.setColor(self.ui.tab_10.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_10.setPalette(p)
    self.ui.tab_10.setAutoFillBackground(True)
    #
    p = self.ui.tab_11.palette()
    p.setColor(self.ui.tab_11.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_11.setPalette(p)
    self.ui.tab_11.setAutoFillBackground(True)
    #
    p = self.ui.tab_12.palette()
    p.setColor(self.ui.tab_12.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_12.setPalette(p)
    self.ui.tab_12.setAutoFillBackground(True)
    #
    p = self.ui.tab_13.palette()
    p.setColor(self.ui.tab_13.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.tab_13.setPalette(p)
    self.ui.tab_13.setAutoFillBackground(True)
    #
    p = self.ui.wall_control_frame.palette()
    p.setColor(self.ui.wall_control_frame.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.wall_control_frame.setPalette(p)
    self.ui.wall_control_frame.setAutoFillBackground(True)
    #
    p = self.ui.reg_vel_frame.palette()
    p.setColor(self.ui.reg_vel_frame.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.reg_vel_frame.setPalette(p)
    self.ui.reg_vel_frame.setAutoFillBackground(True)
    #
    p = self.ui.log_options.palette()
    p.setColor(self.ui.log_options.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.log_options.setPalette(p)
    self.ui.log_options.setAutoFillBackground(True)
    #
    p = self.ui.frame_5.palette()
    p.setColor(self.ui.frame_5.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_5.setPalette(p)
    self.ui.frame_5.setAutoFillBackground(True)
    #
    p = self.ui.frame_12.palette()
    p.setColor(self.ui.frame_12.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_12.setPalette(p)
    self.ui.frame_12.setAutoFillBackground(True)
    #
    p = self.ui.frame_13.palette()
    p.setColor(self.ui.frame_13.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_13.setPalette(p)
    self.ui.frame_13.setAutoFillBackground(True)
    #
    p = self.ui.frame_8.palette()
    p.setColor(self.ui.frame_8.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_8.setPalette(p)
    self.ui.frame_8.setAutoFillBackground(True)
    #
    p = self.ui.frame_3.palette()
    p.setColor(self.ui.frame_3.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_3.setPalette(p)
    self.ui.frame_3.setAutoFillBackground(True)
    #
    p = self.ui.frame_6.palette()
    p.setColor(self.ui.frame_6.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_6.setPalette(p)
    self.ui.frame_6.setAutoFillBackground(True)
    #
    p = self.ui.frame_9.palette()
    p.setColor(self.ui.frame_9.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_9.setPalette(p)
    self.ui.frame_9.setAutoFillBackground(True)
    #
    p = self.ui.frame_10.palette()
    p.setColor(self.ui.frame_10.backgroundRole(), QtCore.Qt.lightGray)
    self.ui.frame_10.setPalette(p)
    self.ui.frame_10.setAutoFillBackground(True)
    # battery and time frame can be yellow or red
    self.ui.frame_batt_time.setAutoFillBackground(True)
    # set start tab
    self.ui.tabPages.setCurrentIndex(self.getIndex(2))
    # set software version number
    #self.ui.robot_me_version.setText("client sw " + robot.clientVersion())
    self.timer.start(50)
    # event connections
    self.ui.main_help.clicked.connect(robot.mainSettingHelp)
    self.ui.main_status_clear.clicked.connect(robot.mainStatusClear)
    self.ui.main_start.clicked.connect(robot.mainSettingStart)
    self.ui.main_stop.clicked.connect(robot.mainSettingStop)
    self.ui.main_status_while_running.stateChanged.connect(robot.statusWhileRunning)
    #self.ui.main_start_2.clicked.connect(robot.mainSettingStart)
    #self.ui.log_flag_apply.clicked.connect(robot.logFlagApply)
    # menu actions
    self.ui.actionQuit.triggered.connect(self.stop)
    self.ui.actionAbout.triggered.connect(self.menuAbout)
    self.ui.actionDebug.triggered.connect(self.menuShowDebug)
    self.ui.actionZ.triggered.connect(self.menuShowZ)
    self.ui.actionWifi.triggered.connect(self.menuShowWiFi)
    self.ui.actionLog.triggered.connect(self.menuShowLog)
    self.ui.actionRobot.triggered.connect(self.menuShowRobot)
    self.ui.actionIMU.triggered.connect(self.menuShowIMU)
    self.ui.actionLine.triggered.connect(self.menuShowLine)
    self.ui.actionDistance.triggered.connect(self.menuShowDist)
    self.ui.actionVelocity.triggered.connect(self.menuShowVelocity)
    self.ui.actionPosition.triggered.connect(self.menuShowPosition)
    self.ui.actionTurn.triggered.connect(self.menuShowTurn)
    self.ui.actionBalance.triggered.connect(self.menuShowBalance)
    self.ui.actionSS.triggered.connect(self.menuShowSS)
    self.ui.actionFollow_line.triggered.connect(self.menuShowFollowLine)
    self.ui.actionMission.triggered.connect(self.menuShowMission)
    self.ui.actionFollow_wall.triggered.connect(self.menuShowFollowWall)
    self.ui.actionShow_all.triggered.connect(self.menuShowAll)
    self.ui.actionHide_most.triggered.connect(self.menuHideMost)
    #
    self.ui.connect_usb.clicked.connect(robot.connect_usb_changed)
    self.ui.connect_wifi.clicked.connect(robot.connect_wifi_changed)
    # regulator turn edit
    self.ui.reg_turn_use.stateChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_use.stateChanged.connect(robot.regTurn.regulatorUseClicked)
    self.ui.reg_turn_LeadFwd.stateChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_LeadFwd.stateChanged.connect(robot.regTurn.regulatorUseClicked)
    self.ui.reg_turn_KP.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_tau_d.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_tau_i.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_alpha.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_ilimit.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_out_limit.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_step_on.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_step_off.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_step_val.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_step_vel.valueChanged.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_step_val.valueChanged.connect(robot.regTurn.stepOrVelValueChanged)
    self.ui.reg_turn_step_vel.valueChanged.connect(robot.regTurn.stepOrVelValueChanged)
    self.ui.reg_turn_apply.clicked.connect(robot.regulatorParamTurnApply)
    self.ui.reg_turn_read.clicked.connect(robot.regTurn.regulatorParamRead)
    self.ui.reg_turn_start.clicked.connect(robot.regTurnStart)
    self.ui.reg_turn_edit.clicked.connect(robot.regTurn.dataChangedManually)
    self.ui.reg_turn_helpbox.clicked.connect(robot.regTurn.helpbox)
    # regulator follow wall
    self.ui.reg_wall_LeadFwd.stateChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_LeadFwd.stateChanged.connect(robot.irDist.regulatorUseClicked)
    self.ui.wall_follow_left.stateChanged.connect(robot.irDist.dataChangedManually)
    self.ui.wall_follow_left.stateChanged.connect(robot.irDist.regulatorUseClicked)
    self.ui.wall_sensor_on.stateChanged.connect(robot.irDist.dataChangedManually)
    self.ui.wall_sensor_on.stateChanged.connect(robot.irDist.regulatorUseClicked)
    self.ui.reg_wall_KP.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_tau_d.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_tau_i.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_alpha.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_ilimit.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_out_limit.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_step_time.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_step_from.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_step_to.valueChanged.connect(robot.irDist.dataChangedManually)
    self.ui.reg_wall_step_vel.valueChanged.connect(robot.irDist.dataChangedManually)
    #self.ui.reg_wall_step_from.valueChanged.connect(robot.irDist.stepOrVelValueChanged)
    #self.ui.reg_wall_step_to.valueChanged.connect(robot.irDist.stepOrVelValueChanged)
    #self.ui.reg_wall_step_vel.valueChanged.connect(robot.irDist.stepOrVelValueChanged)
    self.ui.wall_apply.clicked.connect(robot.irDist.paramApply)
    self.ui.wall_cancel.clicked.connect(robot.irDist.paramCancel)
    self.ui.wall_start.clicked.connect(robot.irDist.start)
    self.ui.wall_edit.clicked.connect(robot.irDist.dataChangedManually)
    self.ui.ir_apply.clicked.connect(robot.irDist.paramApplyCal) 
    self.ui.ir_cancel.clicked.connect(robot.irDist.paramCancelCal)
    self.ui.ir_edit.clicked.connect(robot.irDist.dataEditCal)
    self.ui.ir_d1_20cm.valueChanged.connect(robot.irDist.dataEditCal)
    self.ui.ir_d1_80cm.valueChanged.connect(robot.irDist.dataEditCal)
    self.ui.ir_d2_20cm.valueChanged.connect(robot.irDist.dataEditCal)
    self.ui.ir_d2_80cm.valueChanged.connect(robot.irDist.dataEditCal)
    self.ui.wall_helpbox.clicked.connect(robot.irDist.helpbox)
    # regulator robot forward position 
    self.ui.reg_pos_use.stateChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_use.stateChanged.connect(robot.regPosition.regulatorUseClicked)
    self.ui.reg_pos_LeadFwd.stateChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_LeadFwd.stateChanged.connect(robot.regPosition.regulatorUseClicked)
    self.ui.reg_pos_KP.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_tau_d.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_tau_i.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_alpha.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_ilimit.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_out_limit.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_step_time.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_step_from.valueChanged.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_step_to.valueChanged.connect(robot.regPosition.dataChangedManually)
    #self.ui.reg_pos_step_vel.valueChanged.connect(robot.regPosition.dataChangedManually)
    #self.ui.reg_pos_step_val.valueChanged.connect(robot.regPosition.stepOrVelValueChanged)
    #self.ui.reg_pos_step_vel.valueChanged.connect(robot.regPosition.stepOrVelValueChanged)
    self.ui.reg_pos_apply.clicked.connect(robot.regPosition.regulatorParamApply)
    self.ui.reg_pos_cancel.clicked.connect(robot.regPosition.regulatorParamCancel)
    self.ui.reg_pos_start.clicked.connect(robot.regPosition.regStart)
    self.ui.reg_pos_edit.clicked.connect(robot.regPosition.dataChangedManually)
    self.ui.reg_pos_helpbox.clicked.connect(robot.regPosition.helpbox)
    # regulator velocoty
    self.ui.reg_vel_use.stateChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_use.stateChanged.connect(robot.regVel.regulatorUseClicked)
    self.ui.reg_vel_est_use.stateChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_LeadFwd.stateChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_LeadFwd.stateChanged.connect(robot.regVel.regulatorUseClicked)
    self.ui.reg_vel_KP.valueChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_tau_d.valueChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_tau_i.valueChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_alpha.valueChanged.connect(robot.regVel.dataChangedManually)    
    self.ui.reg_vel_integrate_max.valueChanged.connect(robot.regVel.dataChangedManually)    
    self.ui.reg_vel_steptime.valueChanged.connect(robot.regVel.dataChangedManually)    
    self.ui.reg_vel_step_from.valueChanged.connect(robot.regVel.stepValueChanged)    
    self.ui.reg_vel_step_to.valueChanged.connect(robot.regVel.stepValueChanged)    
    self.ui.reg_vel_step_from.valueChanged.connect(robot.regVel.dataChangedManually)    
    self.ui.reg_vel_step_to.valueChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_acc_limit.valueChanged.connect(robot.regVel.dataChangedManually)
    self.ui.vel_acc_limit_use.clicked.connect(robot.regVel.dataChangedManually)
    self.ui.vel_acc_limit_use.clicked.connect(robot.regVel.accLimitUse)
    self.ui.reg_vel_volt_limit.valueChanged.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_apply.clicked.connect(robot.regulatorParamVelApply)
    self.ui.reg_vel_read.clicked.connect(robot.regVel.regulatorParamRead)
    self.ui.reg_vel_edit.clicked.connect(robot.regVel.dataChangedManually)
    self.ui.reg_vel_start.clicked.connect(robot.regVelStart)
    self.ui.reg_vel_helpbox.clicked.connect(robot.regVel.helpbox)
    # regulator balance
    self.ui.reg_bal_use.stateChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_use.stateChanged.connect(robot.regBal.regulatorUseClicked)
    self.ui.reg_bal_LeadFwd.stateChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_LeadGyro.stateChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_LeadGyro.stateChanged.connect(robot.regBal.regulatorUseClicked)
    self.ui.reg_bal_LeadFwd.stateChanged.connect(robot.regBal.regulatorUseClicked)
    self.ui.reg_bal_KP.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_tau_d.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_tau_i.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_alpha.valueChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_bal_integrate_max.valueChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_bal_out_limit.valueChanged.connect(robot.regBal.dataChangedManually)    
    # balance buttons
    self.ui.reg_bal_apply.clicked.connect(robot.regulatorParamBalApply)
    self.ui.reg_bal_read.clicked.connect(robot.regBal.regulatorParamRead)
    self.ui.reg_bal_start.clicked.connect(robot.regBalStart)
    self.ui.reg_bal_edit.clicked.connect(robot.regBal.dataChangedManually)
    self.ui.reg_bal_helpbox.clicked.connect(robot.regBal.helpbox)
    # regulator balance speed
    self.ui.reg_balvel_use.stateChanged.connect(robot.regBal.regulatorUseClicked)
    self.ui.reg_mvel_LeadFwd.stateChanged.connect(robot.regBal.regulatorUseClicked)
    self.ui.reg_mvel_KP.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_mvel_tau_d.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_mvel_tau_i.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_mvel_alpha.valueChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_mvel_zeta.valueChanged.connect(robot.regBal.dataChangedManually)
    self.ui.reg_mvel_integrate_max.valueChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_mvel_LeadFwd.stateChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_mvel_out_limit.valueChanged.connect(robot.regBal.dataChangedManually)    
    # balance step
    self.ui.reg_bal_steptime.valueChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_bal_step_from.valueChanged.connect(robot.regBal.stepValueChanged)    
    self.ui.reg_bal_step_to.valueChanged.connect(robot.regBal.stepValueChanged)    
    self.ui.reg_bal_step_from.valueChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_balvel_use.stateChanged.connect(robot.regBal.dataChangedManually)    
    self.ui.reg_bal_step_to.valueChanged.connect(robot.regBal.dataChangedManually)
    # regulator balance state space
    self.ui.reg_bal_ss_use.stateChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_use.stateChanged.connect(robot.regSSBal.regulatorUseClicked)
    self.ui.reg_bal_ss_k_tilt.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_k_gyro.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_k_pos.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_k_vel.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_k_motor.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_out_limit.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_steptime.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_step_from.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_step_to.valueChanged.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_bal_ss_step.valueChanged.connect(robot.regSSBal.dataChangedManually)
    # regulator balance state space buttons
    self.ui.reg_ss_apply.clicked.connect(robot.regulatorParamSSApply)
    self.ui.reg_ss_read.clicked.connect(robot.regSSBal.regulatorParamRead)
    self.ui.reg_ss_start.clicked.connect(robot.regSSBalStart)
    self.ui.reg_ss_edit.clicked.connect(robot.regSSBal.dataChangedManually)
    self.ui.reg_ss_helpbox.clicked.connect(robot.regSSBal.helpbox)
    # gyro
    self.ui.imu_gyro_calibrate.clicked.connect(robot.doGyroOffset)
    # line sensor
    self.ui.line_helpbox.clicked.connect(robot.lineSensor.helpbox)
    self.ui.reg_ls_apply.clicked.connect(robot.lineSensor.applySettings)
    self.ui.reg_ls_cancel.clicked.connect(robot.lineSensor.regulatorParamCancel)
    self.ui.reg_ls_start.clicked.connect(robot.lineSensor.startLsMission)
    self.ui.reg_ls_edit.clicked.connect(robot.lineSensor.dataChangedManually)
    self.ui.line_disp_max_value.valueChanged.connect(robot.lineSensor.max_value_changed)
    #self.ui.ls_use_sensor.clicked.connect(robot.lineSensor.dataChangedManually)
    self.ui.ls_sensor_on.clicked.connect(robot.lineSensor.sensorOnClicked)
    self.ui.ls_use_sensor.clicked.connect(robot.lineSensor.setWhiteLine)
    self.ui.ls_line_white.clicked.connect(robot.lineSensor.setWhiteLine)
    self.ui.ls_power_auto.clicked.connect(robot.lineSensor.setWhiteLine)
    self.ui.ls_power_high.clicked.connect(robot.lineSensor.setWhiteLine)
    self.ui.ls_calibrate_white.clicked.connect(robot.lineSensor.calibrateWhite)
    self.ui.ls_calibrate_black.clicked.connect(robot.lineSensor.calibrateBlack)
    self.ui.reg_turn_step_val_2.valueChanged.connect(robot.lineSensor.stepOrVelValueChanged)
    self.ui.reg_turn_step_vel_2.valueChanged.connect(robot.lineSensor.stepOrVelValueChanged)
    #self.ui.reg_turn_use_2.stateChanged.connect(robot.lineSensor.dataChangedManually)
    #self.ui.reg_turn_use_2.stateChanged.connect(robot.lineSensor.regulatorUseClicked)
    self.ui.reg_turn_LeadFwd_2.stateChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_LeadFwd_2.stateChanged.connect(robot.lineSensor.regulatorUseClicked)
    self.ui.reg_turn_KP_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_tau_d_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_tau_i_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_alpha_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_ilimit_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_out_limit_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_step_on_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_step_from_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_step_val_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.reg_turn_step_vel_2.valueChanged.connect(robot.lineSensor.dataChangedManually)
    self.ui.ls_follow_left.clicked.connect(robot.lineSensor.dataChangedManually)
    # main status
    self.ui.main_send_cmd.clicked.connect(robot.sendCmd)
    # - when return is pressed in line edit
    self.ui.main_send_txt.returnPressed.connect(robot.sendCmd)
    self.ui.main_mission_2.valueChanged.connect(robot.mission.dataChangedManually)
    self.ui.main_mission_2.valueChanged.connect(robot.mainMissionChanged2)
    self.ui.main_push_interval.valueChanged.connect(robot.mainPushInterval)
    self.ui.main_status_stop.clicked.connect(robot.mainStatusStop)
    self.ui.main_status_start.clicked.connect(robot.mainStatusStart)
    # config save/load
    self.ui.config_file_load.clicked.connect(robot.configurationFileLoadDef)
    self.ui.config_file_save.clicked.connect(robot.configurationFileSaveDef)
    self.ui.config_robot_save.clicked.connect(robot.configurationRobotSave)
    self.ui.actionLoad_configuration.triggered.connect(robot.configurationFileLoadDef)
    self.ui.actionLoad_configuration_from.triggered.connect(robot.configurationFileLoadFrom)
    self.ui.actionSave_configuration.triggered.connect(robot.configurationFileSaveDef)
    self.ui.actionSave_configuration_as.triggered.connect(robot.configurationFileSaveAs)
    # robot
    self.ui.robot_pose_reset.clicked.connect(robot.robotPoseReset)
    #log 
    self.ui.log_lac.stateChanged.connect(robot.setLogFlag_lac)
    self.ui.log_lbt.stateChanged.connect(robot.setLogFlag_lbt)
    self.ui.log_line.stateChanged.connect(robot.setLogFlag_line)
    self.ui.log_distance.stateChanged.connect(robot.setLogFlag_dist)
    self.ui.log_turn_rate.stateChanged.connect(robot.setLogFlag_ltr)
    self.ui.log_lbc.stateChanged.connect(robot.setLogFlag_lbc)
    self.ui.log_lct.stateChanged.connect(robot.setLogFlag_lct)
    self.ui.log_lgy.stateChanged.connect(robot.setLogFlag_lgy)
    self.ui.log_lma.stateChanged.connect(robot.setLogFlag_lma)
    self.ui.log_lme.stateChanged.connect(robot.setLogFlag_lme)
    self.ui.log_lmr.stateChanged.connect(robot.setLogFlag_lmr)
    self.ui.log_lms.stateChanged.connect(robot.setLogFlag_lms)
    self.ui.log_lmv.stateChanged.connect(robot.setLogFlag_lmv)
    self.ui.log_lvr.stateChanged.connect(robot.setLogFlag_lvr)
    self.ui.log_lpo.stateChanged.connect(robot.setLogFlag_lpo)
    self.ui.log_lex.stateChanged.connect(robot.setLogFlag_lex)
    self.ui.log_turn_rate.stateChanged.connect(robot.setLogFlag_ltr)
    self.ui.log_get.clicked.connect(robot.logGet)
    self.ui.log_interval.valueChanged.connect(robot.setLogInterval)
    self.ui.log_allow.clicked.connect(robot.setLogInterval)
    self.ui.log_get_filename.clicked.connect(robot.setLogFileName) 
    self.ui.log_save.clicked.connect(robot.logSave)
    self.ui.log_clear.clicked.connect(robot.log.logClear)
    # mission
    self.ui.mission_get_filename.clicked.connect(robot.mission.setMissionFileName)
    self.ui.mission_file_save.clicked.connect(robot.mission.saveMissionToFile)
    self.ui.mission_file_load.clicked.connect(robot.mission.loadMissionFromFile)
    self.ui.mission_robot_save.clicked.connect(robot.mission.sendToRobot)
    self.ui.mission_syntax_check.clicked.connect(robot.mission.checkMission)
    self.ui.mission_robot_load.clicked.connect(robot.mission.getFromRobot)
    self.ui.mission_help.clicked.connect(robot.mission.helpbox)
    # robot ID etc
    self.ui.robot_wheel_radius_right.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.robot_edit.clicked.connect(robot.info.dataChangedManually)
    self.ui.robot_cancel.clicked.connect(robot.info.cancelEdit)
    self.ui.robot_wheel_radius_left.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.robot_pulse_per_rev.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.robot_gear.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.robot_base.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.robot_id.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.robot_balance_offset.valueChanged.connect(robot.info.dataChangedManually)
    self.ui.save_id_on_robot.clicked.connect(robot.robotIdApply)
    self.ui.robot_on_battery.clicked.connect(robot.info.dataChangedManually)
    self.ui.robot_battery_idle_volt.valueChanged.connect(robot.info.dataChangedManually)
    # wifi
    self.ui.wifi_apply.clicked.connect(robot.info.wifiSendData)
    self.ui.wifi_get.clicked.connect(robot.info.wifiGetData)
    self.ui.wifi_edit.clicked.connect(robot.info.wifiEdit)
    self.ui.wifi_save_MAC_list.clicked.connect(robot.info.wifiSaveMacList)
    #
    # graph test
    # robot.imu.initGraph()
    if (False):
      print("graph test start")
      #area = DockArea(self.ui.reg_vel_frame)
      # mymw.ui.frame_10. setCentralWidget(area)
      #d3 = Dock("Dock3", size=(10,100))
      pw = pg.PlotWidget(name='Plot1')  ## giving the plots names allows us to link their axes together
      self.ui.robot_graph_layout.addWidget(pw)
      pw.plot(np.random.normal(size=100))
      print("graph test end")
      # graph test end
    print("Running ...")
    #
    #
  def stop(self):
    print("REGBOT: Stopping\n")
    robot.terminate();
  def closeEvent(self, event):
    self.stop()
  def menuAbout(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    about_box = QtGui.QMessageBox(mymw)
    about_box.setText('''<p><span style=" font-size:20pt;">
               <a href="http://www.dtu.dk">DTU</a>
               <a href="http://www.elektro.dtu.dk"> Electro</a>
               <a href="http://rsewiki.elektro.dtu.dk/index.php/Regbot"> REGBOT<a></span></p>
               <p><span style=" font-size:10pt;">Robot used 
               in <a href="http://www.kurser.dtu.dk/31300.aspx?menulanguage=en-GB">31300</a>
               and <a href="http://www.kurser.dtu.dk/31300.aspx?menulanguage=en-GB">31301</a> Linear control 1 course.</span></p>
               <p><span style=" font-size:10pt;">This is the robot configuration GUI</span></p>
               <p><span style=" font-size:10pt;">Version ''' + robot.clientVersion() + ''' (''' + robot.clientVersionDate() + ''')</span></p>
               <p><span style=" font-size:10pt;">Last connected robot</span></p>
               <p><span style=" font-size:10pt;">''' + robot.info.robot.name + ''' (''' + str(robot.info.robot.robotID) + ''') version ''' + str(robot.info.robot.version) + '''.</span></p>
               <p><span style=" font-size:10pt;"><a href="http://rsewiki.elektro.dtu.dk/index.php/Regbot">Software info</a> </span></p>
               <p><span style=" font-size:10pt;">(contact: jca@elektro.dtu.dk)</span></p>''');
    about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
    about_box.setWindowTitle("regbot about")
    about_box.exec_()
    #QtGui.QMessageBox.about(self, 'About Message',
            #'''<p><span style=" font-size:20pt;">DTU Electro REGBOT</span></p>
               #<p><span style=" font-size:10pt;">Robot intended for 31300 and 31301 Linear control 1 course.</span></p>
               #<p><span style=" font-size:10pt;">This is the robot configuration GUI</span></p>
               #<p><span style=" font-size:10pt;">Version ''' + gg[1] + '''</span></p>
               #<p><span style=" font-size:10pt;"><a href="http://rsewiki.elektro.dtu.dk">Software info</a> </span></p>
               #<p><span style=" font-size:10pt;">(contact: jca@elektro.dtu.dk)</span></p>''')    
  #
  ### get index of tab with this preferred position (if all tabs were shown)
  def getIndex(self, tab):
    i = 0
    if (self.ui.actionDebug.isChecked()):
      i = i + 1
    if (tab <= 1):
      return i
    if (self.ui.actionZ.isChecked()):
      i = i + 1
    if (tab <= 2):
      return i
    if (self.ui.actionWifi.isChecked()):
      i = i + 1
    if (tab <= 3):
      return i
    if (self.ui.actionLog.isChecked()):
      i = i + 1
    if (tab <= 4):
      return i
    if (self.ui.actionRobot.isChecked()):
      i = i + 1
    if (tab <= 5):
      return i
    if (self.ui.actionIMU.isChecked()):
      i = i + 1
    if (tab <= 6):
      return i
    if (self.ui.actionLine.isChecked()):
      i = i + 1
    if (tab <= 7):
      return i
    if (self.ui.actionDistance.isChecked()):
      i = i + 1
    if (tab <= 8):
      return i
    if (self.ui.actionVelocity.isChecked()):
      i = i + 1
    if (tab <= 9):
      return i
    if (self.ui.actionPosition.isChecked()):
      i = i + 1
    if (tab <= 10):
      return i
    if (self.ui.actionTurn.isChecked()):
      i = i + 1
    if (tab <= 11):
      return i
    if (self.ui.actionBalance.isChecked()):
      i = i + 1
    if (tab <= 12):
      return i
    if (self.ui.actionSS.isChecked()):
      i = i + 1
    if (tab <= 13):
      return i
    if (self.ui.actionFollow_line.isChecked()):
      i = i + 1
    if (tab <= 14):
      return i
    if (self.ui.actionFollow_wall.isChecked()):
      i = i + 1
    if (tab <= 15):
      return i
    if (self.ui.actionMission.isChecked()):
      i = i + 1
    return i

  def menuShowMission(self):
    if (self.ui.actionMission.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(14), self.ui.tab_6, "Mission")
      self.ui.tabPages.setCurrentIndex(self.getIndex(14))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_6)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no IMU tab found")
    pass
  def menuShowFollowWall(self):
    if (self.ui.actionFollow_wall.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(13), self.ui.tab_13, "Follow wall")
      self.ui.tabPages.setCurrentIndex(self.getIndex(13))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_13)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowFollowLine(self):
    if (self.ui.actionFollow_line.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(12), self.ui.tab_9, "Follow edge")
      self.ui.tabPages.setCurrentIndex(self.getIndex(12))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_9)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowSS(self):
    if (self.ui.actionSS.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(11), self.ui.tab_8, "SS")
      self.ui.tabPages.setCurrentIndex(self.getIndex(11))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_8)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowBalance(self):
    if (self.ui.actionBalance.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(10), self.ui.tab_5, "Balance")
      self.ui.tabPages.setCurrentIndex(self.getIndex(10))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_5)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowTurn(self):
    if (self.ui.actionTurn.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(9), self.ui.tab_4, "Turn")
      self.ui.tabPages.setCurrentIndex(self.getIndex(9))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_4)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowVelocity(self):
    if (self.ui.actionVelocity.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(7), self.ui.speed_ctrl, "Velocity")
    else:
      t = self.ui.tabPages.indexOf(self.ui.speed_ctrl)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowPosition(self):
    if (self.ui.actionPosition.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(8), self.ui.tab_12, "Position")
      self.ui.tabPages.setCurrentIndex(self.getIndex(8))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_12)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no Debug tab found")
    pass
  def menuShowDist(self):
    if (self.ui.actionDistance.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(6), self.ui.tab_11, "IR Distance")
      self.ui.tabPages.setCurrentIndex(self.getIndex(6))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_11)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no Debug tab found")
    pass
  def menuShowLine(self):
    if (self.ui.actionLine.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(5), self.ui.tab_10, "Edge")
      self.ui.tabPages.setCurrentIndex(self.getIndex(5))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_10)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowIMU(self):
    if (self.ui.actionIMU.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(4), self.ui.tab_3, "IMU")
      self.ui.tabPages.setCurrentIndex(self.getIndex(4))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_3)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowRobot(self):
    if (self.ui.actionRobot.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(3), self.ui.tab_2, "Robot")
      self.ui.tabPages.setCurrentIndex(self.getIndex(3))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_2)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
    pass
  def menuShowLog(self):
    if (self.ui.actionLog.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(2), self.ui.tab, "Log")
      self.ui.tabPages.setCurrentIndex(self.getIndex(2))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no Log tab found")
    pass
  def menuShowZ(self):
    if (self.ui.actionZ.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(1), self.ui.tab_7, "Z")
      self.ui.tabPages.setCurrentIndex(self.getIndex(1))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_7)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no Z tab found")
    pass
  def menuShowWiFi(self):
    if (self.ui.actionWifi.isChecked()):
      self.ui.tabPages.insertTab(self.getIndex(1), self.ui.tab_14, "Wifi")
      self.ui.tabPages.setCurrentIndex(self.getIndex(1))
    else:
      t = self.ui.tabPages.indexOf(self.ui.tab_14)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no WiFi tab found")
    pass
  def menuShowDebug(self):
    if (self.ui.actionDebug.isChecked()):
      self.ui.tabPages.insertTab(0, self.ui.main, "Debug")
      self.ui.tabPages.setCurrentIndex(0)
    else:
      t = self.ui.tabPages.indexOf(self.ui.main)
      if t >= 0:
        self.ui.tabPages.removeTab(t)
      else:
        print("no Debug tab found")
    pass
  def menuShowAll(self):
    if not self.ui.actionFollow_wall.isChecked():
      self.ui.actionFollow_wall.setChecked(True)
      self.menuShowFollowWall()
    if not self.ui.actionDebug.isChecked():
      self.ui.actionDebug.setChecked(True)
      self.menuShowDebug()
    if not self.ui.actionZ.isChecked():
      self.ui.actionZ.setChecked(True)
      self.menuShowZ()
    if not self.ui.actionWifi.isChecked():
      self.ui.actionWifi.setChecked(True)
      self.menuShowWiFi()
    if not self.ui.actionLog.isChecked():
      self.ui.actionLog.setChecked(True)
      self.menuShowLog()
    if not self.ui.actionRobot.isChecked():
      self.ui.actionRobot.setChecked(True)
      self.menuShowRobot()
    if not self.ui.actionIMU.isChecked():
      self.ui.actionIMU.setChecked(True)
      self.menuShowIMU()
    if not self.ui.actionLine.isChecked():
      self.ui.actionLine.setChecked(True)
      self.menuShowLine()
    if not self.ui.actionDistance.isChecked():
      self.ui.actionDistance.setChecked(True)
      self.menuShowDist()
    if not self.ui.actionVelocity.isChecked():
      self.ui.actionVelocity.setChecked(True)
      self.menuShowVelocity()
    if not self.ui.actionPosition.isChecked():
      self.ui.actionPosition.setChecked(True)
      self.menuShowPosition()
    if not self.ui.actionTurn.isChecked():
      self.ui.actionTurn.setChecked(True)
      self.menuShowTurn()
    if not self.ui.actionBalance.isChecked():
      self.ui.actionBalance.setChecked(True)
      self.menuShowBalance()
    if not self.ui.actionSS.isChecked():
      self.ui.actionSS.setChecked(True)
      self.menuShowSS()
    if not self.ui.actionFollow_line.isChecked():
      self.ui.actionFollow_line.setChecked(True)
      self.menuShowFollowLine()
    if not self.ui.actionMission.isChecked():
      self.ui.actionMission.setChecked(True)
      self.menuShowMission()
    pass
  def menuHideMost(self):
    if self.ui.actionFollow_wall.isChecked():
      self.ui.actionFollow_wall.setChecked(False)
      self.menuShowFollowWall()
    if self.ui.actionDebug.isChecked():
      self.ui.actionDebug.setChecked(False)
      self.menuShowDebug()
    if self.ui.actionZ.isChecked():
      self.ui.actionZ.setChecked(False)
      self.menuShowZ()
    if self.ui.actionWifi.isChecked():
      self.ui.actionWifi.setChecked(False)
      self.menuShowWiFi()
    if not self.ui.actionLog.isChecked():
      self.ui.actionLog.setChecked(True)
      self.menuShowLog()
    if not self.ui.actionRobot.isChecked():
      self.ui.actionRobot.setChecked(True)
      self.menuShowRobot()
    if self.ui.actionIMU.isChecked():
      self.ui.actionIMU.setChecked(False)
      self.menuShowIMU()
    if self.ui.actionLine.isChecked():
      self.ui.actionLine.setChecked(False)
      self.menuShowLine()
    if self.ui.actionDistance.isChecked():
      self.ui.actionDistance.setChecked(False)
      self.menuShowDist()
    if not self.ui.actionVelocity.isChecked():
      self.ui.actionVelocity.setChecked(True)
      self.menuShowVelocity()
    if self.ui.actionPosition.isChecked():
      self.ui.actionPosition.setChecked(False)
      self.menuShowPosition()
    if not self.ui.actionTurn.isChecked():
      self.ui.actionTurn.setChecked(True)
      self.menuShowTurn()
    if self.ui.actionBalance.isChecked():
      self.ui.actionBalance.setChecked(False)
      self.menuShowBalance()
    if self.ui.actionSS.isChecked():
      self.ui.actionSS.setChecked(False)
      self.menuShowSS()
    if self.ui.actionFollow_line.isChecked():
      self.ui.actionFollow_line.setChecked(False)
      self.menuShowFollowLine()
    if not self.ui.actionMission.isChecked():
      self.ui.actionMission.setChecked(True)
      self.menuShowMission()
    pass
    

#http://rsewiki.elektro.dtu.dk/index.php/Regbot
#
# Create application and main window
#
if __name__ == '__main__':
  myapp = QtGui.QApplication(sys.argv)
  #myapp = QApplication(sys.argv)
  mymw = UMainWindow()
  robot.configurationFileLoadDef(True)
  mymw.setWindowIcon(QtGui.QIcon("dtulogoicon_123x123.png"))
  # mymw.setIcon.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
  robot.imu.initGraph()
  robot.drive.initGraph()
  mymw.show()
  #myapp.exec()
  sys.exit(myapp.exec_())
