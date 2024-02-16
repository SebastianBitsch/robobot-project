#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* IR distance sensor functions
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
#import numpy as np
#import pyqtgraph as pg
import time



class UIRDistance(object):
  # sensor control
  sensorOn = False
  sensorInstalled = False
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
  nextCommand = 0
  inEdit2 = False # sensor buttons
  # help
  about_box = None
  # resource lock
  lock = threading.RLock()
  lastTab = ""
  waitingForData = False
  lastDataRequestTime = time.time()

  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui

  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "irc":
        self.distS1 = float(gg[1])
        self.distS2 = float(gg[2])
        self.irRaw[0] = int(gg[3],0)
        self.irRaw[1] = int(gg[4],0)
        self.irCal20cm[0] = int(gg[5],0)
        self.irCal80cm[0] = int(gg[6],0)
        self.irCal20cm[1] = int(gg[7],0)
        self.irCal80cm[1] = int(gg[8],0)
        self.sensorOn = int(gg[9],0)
        if (len(gg) > 10):
          self.sensorInstalled = int(gg[10],0)
        self.dataDistRead = True
        self.waitingForData = False
      else:
        used = False
    except:
      print("IR sensor: data read error - skipped a " + gg[0])
      pass
    self.lock.release()
    return used
  def timerUpdate(self):
    self.lock.acquire()
    if (self.dataDistRead):
      self.dataDistRead = False
      self.inUpdate = True
      self.ui.ir_d1_meters.setText(str(self.distS1))
      self.ui.ir_d2_meters.setText(str(self.distS2))
      self.ui.ir_bar_1.setValue(self.irRaw[0])
      self.ui.ir_bar_2.setValue(self.irRaw[1])
      self.ui.ir_d1_raw.setValue(self.irRaw[0])
      self.ui.ir_d2_raw.setValue(self.irRaw[1])
      if not self.inEdit2:
        self.ui.ir_d1_20cm.setValue(self.irCal20cm[0])
        self.ui.ir_d2_20cm.setValue(self.irCal20cm[1])
        self.ui.ir_d1_80cm.setValue(self.irCal80cm[0])
        self.ui.ir_d2_80cm.setValue(self.irCal80cm[1])
        self.ui.checkBox_ir_use.setChecked(self.sensorOn)
        self.ui.checkBox_ir_installed.setChecked(self.sensorInstalled)
      # enable buttons
      self.ui.ir_apply.setEnabled(self.inEdit2)
      self.ui.ir_edit.setEnabled(not self.inEdit2)
      self.ui.ir_cancel.setEnabled(self.inEdit2)
      #print("distance is " + str(self.distS1) + ", " + str(self.distS2))
      self.inUpdate = False
    # request new data
    if self.robot.info.talkToBridge:
      if self.lastTab != self.robot.currentTab:
        if self.robot.currentTab == "ir":
          # subscribe 4 data from bridge
          self.robot.conWrite("irc subscribe 2\n") # IR data all
        elif self.lastTab == "ir":
          # unsubscribe
          self.robot.conWrite("irc subscribe 0\n") # IR data (all)
          pass
        self.lastTab = self.robot.currentTab

    if (self.robot.currentTab == "ir" and not self.robot.info.talkToBridge):
      if (self.robot.wifiConnected or self.robot.isConnected):
        if not self.waitingForData and time.time() - self.lastDataRequestTime > 0.2:
          self.robot.conWrite("v0\n") # limit black
          self.waitingForData = True
          self.lastDataRequestTime = time.time()
        elif time.time() - self.lastDataRequestTime > 1.5:
          # request timeout
          self.waitingForData = False

    self.lock.release()
  #
  def dataEditCal(self):
    if (not self.robot.timerUpdate):
      self.lock.acquire()
      self.inEdit2 = True
      self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def paramCancelCal(self):
    # cancel button pressed
    self.inEdit2 = False
  #def helpbox(self):
    #if (self.about_box == None):
      #self.about_box = QtGui.QMessageBox(self.robot.parent)
      #self.about_box.setText('''<p><span style=" font-size:20pt;">
                #Distance sensor</span></p>
                #<p>
                #May in the future be used for wall following
                #</p>''');
      #self.about_box.setWindowTitle("regbot motor velocity")
      #self.about_box.setWindowModality(QtCore.Qt.NonModal)
    #self.about_box.show()
  #
  def paramApplyCal(self):
    self.robot.conWrite("irc=%g %g %g %g %d %d\n" % (
        self.ui.ir_d1_20cm.value(), 
        self.ui.ir_d1_80cm.value(), 
        self.ui.ir_d2_20cm.value(), 
        self.ui.ir_d2_80cm.value(),
        self.ui.checkBox_ir_use.isChecked(),
        self.ui.checkBox_ir_installed.isChecked()
        ))
    self.inEdit2 = False

  def justConnected(self):
    self.lastTab = "none"
