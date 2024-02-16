#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Line sensor functions
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
#import pyqtgraph as pg
import time


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
  wideSensor = False;
  swapLeftRight = False;
  # crossing
  crossingDetectLimit = 4;
  crossingWhite = False
  crossingBlack = False
  crossingWhiteCnt = 0
  #crossingBlackCnt = 0
  power_high = False
  power_auto = True
  # management
  dataReadLiv = False
  dataReadLip = False
  #dataReadCtrl = False
  #dataReadZ = False
  #dataReadZI = False
  dataReadbw = True
  #
  inUpdate = False
  inEdit = False
  nextDataRequest = 1
  lastDataRequestTime = time.time()
  lastTab = ""
  waitingForData = False
  # 
  #regActive = True
  #regKp = 0.0
  #regTauD = 0.0
  #regTauI = 0.0
  #regAlpha = 0.0
  #regIMax = 0.0
  #regStepOn = 0
  #regStepFrom = 0
  #regStepTo = 0 # step value in rad/s or 
  #regStepVel = 0 # velocity base
  #regTurnLeadFwd = True
  #regOutMax = 100
  #
  #regulZNumer = [1.0, 0.0]
  #regulZDenom = [0.0, 0.0]
  #regulZINumer = [1.0, 0.0]
  #regulZIDenom = [0.0, 0.0]
  #
  about_box = None
  # resource lock
  lock = threading.RLock()

  def __init__(self, robot, parent):
    self.robot = robot
    self.ui = robot.ui
    self.parent = parent

  def readData(self, gg):
    used = True
    self.lock.acquire()
    try:
      if gg[0] == "liv":
        # actual data
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
        self.waitingForData = False
      elif gg[0] == "liw":
        # white calibration
        self.lineMaxWhite[0] = int(gg[1],0)
        self.lineMaxWhite[1] = int(gg[2],0)
        self.lineMaxWhite[2] = int(gg[3],0)
        self.lineMaxWhite[3] = int(gg[4],0)
        self.lineMaxWhite[4] = int(gg[5],0)
        self.lineMaxWhite[5] = int(gg[6],0)
        self.lineMaxWhite[6] = int(gg[7],0)
        self.lineMaxWhite[7] = int(gg[8],0)
        self.dataReadbw = True
        self.waitingForData = False
      elif gg[0] == "lib":
        # black calibration
        self.lineMaxBlack[0] = int(gg[1],0)
        self.lineMaxBlack[1] = int(gg[2],0)
        self.lineMaxBlack[2] = int(gg[3],0)
        self.lineMaxBlack[3] = int(gg[4],0)
        self.lineMaxBlack[4] = int(gg[5],0)
        self.lineMaxBlack[5] = int(gg[6],0)
        self.lineMaxBlack[6] = int(gg[7],0)
        self.lineMaxBlack[7] = int(gg[8],0)
        self.dataReadbw = True
        self.waitingForData = False
      elif gg[0] == "lip":
        # derived data
        self.lineUse = int(gg[1],0)
        self.lineWhite = int(gg[2],0)
        self.edgeLeft = float(gg[3])
        self.edgeLeftValid = int(gg[4],0)
        self.edgeRight = float(gg[5])
        self.edgeRightValid = int(gg[6],0)
        self.followLeft = int(gg[7],0)
        self.crossingWhite = int(gg[8],0)
        #self.crossingBlack = int(gg[9],0) # not used anymore
        self.crossingWhiteCnt = int(gg[10],0)
        #self.crossingBlackCnt = int(gg[11],0) # not used anymore
        self.power_high = int(gg[12],0)
        self.power_auto = int(gg[13],0)
        if (len(gg) > 14):
          self.crossingDetectLimit = float(gg[14])
        if (len(gg) > 15):
          self.wideSensor = int(gg[15])
        if (len(gg) > 16):
          self.swapLeftRight = int(gg[16])
        self.dataReadLip = True
        self.waitingForData = False
      else:
        used = False
    except:
      print("URegLine: data read error - skipped a " + gg[0] + " length=" + str(len(gg)))
      pass
    self.lock.release()
    return used
  #
  def timerUpdate(self):
    self.lock.acquire()
    if self.dataReadbw:
      # calibration settings
      self.dataReadbw = False
      self.ui.ls_max_white_1.setText(str(self.lineMaxWhite[0]))
      self.ui.ls_max_white_2.setText(str(self.lineMaxWhite[1]))
      self.ui.ls_max_white_3.setText(str(self.lineMaxWhite[2]))
      self.ui.ls_max_white_4.setText(str(self.lineMaxWhite[3]))
      self.ui.ls_max_white_5.setText(str(self.lineMaxWhite[4]))
      self.ui.ls_max_white_6.setText(str(self.lineMaxWhite[5]))
      self.ui.ls_max_white_7.setText(str(self.lineMaxWhite[6]))
      self.ui.ls_max_white_8.setText(str(self.lineMaxWhite[7]))
      self.ui.ls_max_black_1.setText(str(self.lineMaxBlack[0]))
      self.ui.ls_max_black_2.setText(str(self.lineMaxBlack[1]))
      self.ui.ls_max_black_3.setText(str(self.lineMaxBlack[2]))
      self.ui.ls_max_black_4.setText(str(self.lineMaxBlack[3]))
      self.ui.ls_max_black_5.setText(str(self.lineMaxBlack[4]))
      self.ui.ls_max_black_6.setText(str(self.lineMaxBlack[5]))
      self.ui.ls_max_black_7.setText(str(self.lineMaxBlack[6]))
      self.ui.ls_max_black_8.setText(str(self.lineMaxBlack[7]))
    if (self.dataReadLip):
      self.ui.ls_left_side.setValue(self.edgeLeft)
      self.ui.ls_right_side.setValue(self.edgeRight)
      # Show editable settings
      if (self.dataReadLip and not self.inEdit):
        self.dataReadLip = False
        self.inUpdate = True
        self.ui.ls_use_sensor.setChecked(self.lineUse)
        self.ui.ls_line_white.setChecked(self.lineWhite)
        #self.ui.ls_left_side_valid.setChecked(self.edgeLeftValid)
        #self.ui.ls_right_side_valid.setChecked(self.edgeRightValid)
        ev = 100 - (self.edgeLeft + 2.5) * 20.0
        if (ev < 0):
          ev = 0
        elif ev >100:
          ev = 100
        elif np.isnan(ev):
          ev = 0
        if self.edgeLeftValid:
          self.ui.ls_left_bar.setValue(int(ev))
        else:
          self.ui.ls_left_bar.setValue(-3)
        ev = (self.edgeRight + 2.5) * 20.0
        if (ev < 0):
          ev = 0
        elif ev >100:
          ev = 100
        elif np.isnan(ev):
          ev = 0
        if self.edgeRightValid:
          self.ui.ls_right_bar.setValue(int(ev))
        else:
          self.ui.ls_right_bar.setValue(-3)
        #self.ui.ls_left_bar.setEnabled(self.edgeLeftValid)
        #self.ui.ls_right_bar.setEnabled(self.edgeRightValid)
        #self.ui.ls_follow_left.setChecked(self.followLeft)
        self.ui.ls_crossing_cnt.setText(str(self.crossingWhiteCnt))
        self.ui.ls_line_valid_cnt.setText(str(self.edgeLeftValid))
        #self.ui.ls_crossing_black.setValue(self.crossingBlackCnt)
        #self.ui.frame_ls_crossing_white.setEnabled(self.crossingWhite)
        #self.ui.frame_ls_crossing_black.setEnabled(self.crossingBlack)
        self.ui.ls_power_high.setChecked(self.power_high)
        self.ui.ls_power_auto.setChecked(self.power_auto)
        self.ui.ls_crossing_detect.setValue(self.crossingDetectLimit)
        self.ui.ls_wideSensor.setChecked(self.wideSensor)
        self.ui.ls_swap_left_right.setChecked(self.swapLeftRight)
    # Show line sensor bar values
    mv = self.ui.line_disp_max_value.value() - 1
    if (self.dataReadLiv):
      self.dataReadLiv = False
      self.inUpdate = True
      if (self.lineValue[0] > mv):
        self.ui.line_bar_1.setValue(mv)
      elif self.lineValue[0] < 0:
        self.ui.line_bar_1.setValue(0)
      else:
        self.ui.line_bar_1.setValue(self.lineValue[0])
      self.ui.ls_actual_1.setText(str(self.lineValue[0]))
      if (self.lineValue[1] > mv):
        self.ui.line_bar_2.setValue(mv)
      elif self.lineValue[1] < 0:
        self.ui.line_bar_2.setValue(0)
      else:
        self.ui.line_bar_2.setValue(self.lineValue[1])
      self.ui.ls_actual_2.setText(str(self.lineValue[1]))
      if (self.lineValue[2] > mv):
        self.ui.line_bar_3.setValue(mv)
      elif self.lineValue[2] < 0:
        self.ui.line_bar_3.setValue(0)
      else:
        self.ui.line_bar_3.setValue(self.lineValue[2])
      self.ui.ls_actual_3.setText(str(self.lineValue[2]))
      if (self.lineValue[3] > mv):
        self.ui.line_bar_4.setValue(mv)
      elif self.lineValue[3] < 0:
        self.ui.line_bar_4.setValue(0)
      else:
        self.ui.line_bar_4.setValue(self.lineValue[3])
      self.ui.ls_actual_4.setText(str(self.lineValue[3]))
      if (self.lineValue[4] > mv):
        self.ui.line_bar_5.setValue(mv)
      elif self.lineValue[4] < 0:
        self.ui.line_bar_5.setValue(0)
      else:
        self.ui.line_bar_5.setValue(self.lineValue[4])
      self.ui.ls_actual_5.setText(str(self.lineValue[4]))
      if (self.lineValue[5] > mv):
        self.ui.line_bar_6.setValue(mv)
      elif self.lineValue[5] < 0:
        self.ui.line_bar_6.setValue(0)
      else:
        self.ui.line_bar_6.setValue(self.lineValue[5])
      self.ui.ls_actual_6.setText(str(self.lineValue[5]))
      if (self.lineValue[6] > mv):
        self.ui.line_bar_7.setValue(mv)
      elif self.lineValue[6] < 0:
        self.ui.line_bar_7.setValue(0)
      else:
        self.ui.line_bar_7.setValue(self.lineValue[6])
      self.ui.ls_actual_7.setText(str(self.lineValue[6]))
      if (self.lineValue[7] > mv):
        self.ui.line_bar_8.setValue(mv)
      elif self.lineValue[7] < 0:
        self.ui.line_bar_8.setValue(0)
      else:
        self.ui.line_bar_8.setValue(self.lineValue[7])
      self.ui.ls_actual_8.setText(str(self.lineValue[7]))
    self.inUpdate = False
    # connected to robot
    self.ui.ls_line_white.setEnabled(self.ui.frame_batt_time.isEnabled())
    self.lock.release()
    if self.lastTab != self.robot.currentTab:
      #print("# Edge last tab != currentTab")
      if self.robot.info.talkToBridge:
        #print("# Edge talking to bridge")
        if self.robot.currentTab == "edge":
          #if (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected():
          #print("# Edge is subscribing to data")
          # subscribe to bridge data
          self.robot.conWrite("liw subscribe 6\n") # calibrate white
          self.robot.conWrite("lib subscribe 6\n") # calibrate black
          self.robot.conWrite("liv subscribe 6\n") # sensor values
          self.robot.conWrite("lip subscribe 2\n") # flags and result
        elif self.lastTab == "edge":
          #unsubscribe
          self.robot.conWrite("liw subscribe 0\n") # calibrate white
          self.robot.conWrite("lib subscribe 0\n") # calibrate black
          self.robot.conWrite("liv subscribe 0\n") # sensor values
          self.robot.conWrite("lip subscribe 0\n") # flags and result
      else:
        if self.robot.currentTab == "edge":
          self.robot.conWrite("sub 1 2 3\n") # line sensor data directly from robot
        elif self.lastTab == "edge":
          self.robot.conWrite("sub 0 2 3\n") # unsubscribe line sensor data
      self.lastTab = self.robot.currentTab
    if self.robot.currentTab == "edge":
      if self.robot.wifiConnected  or self.robot.isConnected():
        if time.time() - self.lastDataRequestTime > 1.5:
          # request also limits
          self.nextDataRequest = 1
          self.waitingForData = False
          self.lastDataRequestTime = time.time()
        elif not self.waitingForData:
          self.waitingForData = True
          if (self.nextDataRequest == 1):
            self.robot.conWrite("u9\n") # limit black
          elif (self.nextDataRequest == 2):
            self.robot.conWrite("u10\n") # limit whute
          elif (self.nextDataRequest == 3):
            if (self.ui.ls_show_normalized.isChecked()):
              self.robot.conWrite("u11\n") # values (normalized)
            else:
              self.robot.conWrite("u12\n") # values
          #elif self.nextDataRequest == 4:
            #self.robot.conWrite("u13\n") # edge position
          else:
            # back to just values and results
            self.nextDataRequest = 2
            self.waitingForData = False
          self.nextDataRequest += 1
        #print("Edge: w4d=" + str(self.waitingForData) + " ndr=" + str(self.nextDataRequest))

  #
  def helpbox(self):
    # QMessageBox.about (QWidget parent, QString caption, QString text)
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(self.robot.parent)
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
    v = self.ui.line_disp_max_value.value()
    self.ui.line_bar_1.setMaximum(v)
    self.ui.line_bar_2.setMaximum(v)
    self.ui.line_bar_3.setMaximum(v)
    self.ui.line_bar_4.setMaximum(v)
    self.ui.line_bar_5.setMaximum(v)
    self.ui.line_bar_6.setMaximum(v)
    self.ui.line_bar_7.setMaximum(v)
    self.ui.line_bar_8.setMaximum(v)
    #print("line sensor maximum is " + str(self.ui.line_bar_8.maximum()))
  # when any of the parameters are changed - allow apply button to be pressed
  #def dataChangedManually(self):
    #if (not self.robot.timerUpdate):
      #self.lock.acquire()
      #self.inEdit = True
      ##if ( self.ui.reg_bal_out_limit.value() < 0.5):
        ##self.ui.label_out_limit.setStyleSheet("QLabel { background-color : red; }")
      ##else:
        ##self.ui.label_out_limit.setStyleSheet("QLabel { background-color : lightGray; }")
      ##if (self.ui.reg_balvel_use.isChecked()):
        ##self.ui.reg_bal_step_from.setValue(self.regStepVFrom)
        ##self.ui.reg_bal_step_to.setValue(self.regStepVTo)
      ##else:
        ##self.ui.reg_bal_step_from.setValue(self.regStepFrom)
        ##self.ui.reg_bal_step_to.setValue(self.regStepTo)
      #self.lock.release()
  #def regulatorUseClicked(self):
    #self.configChanged()
    #self.ui.bal_regul_frame.repaint()
  def configChanged(self):
    # load the right value into step from-to widget
    self.inUpdate = True
    self.ui.ls_right_side.setEnabled(self.ui.ls_use_sensor.isChecked())
    self.ui.ls_left_side.setEnabled(self.ui.ls_use_sensor.isChecked())
    self.max_value_changed()
    self.inUpdate = False
    #pass
  def sensorOnClicked(self):
    self.ui.ls_use_sensor.setChecked(self.ui.ls_sensor_on.isChecked())
    self.setWhiteLine()
  def setWhiteLine(self):
    # send white line assumption to robot
    self.robot.conWrite("lip=%d %d %d %d %g %d %d\n" % (
            self.ui.ls_use_sensor.isChecked(),
            self.ui.ls_line_white.isChecked(),
            self.ui.ls_power_high.isChecked(),
            self.ui.ls_power_auto.isChecked(),
            self.ui.ls_crossing_detect.value(),
            self.ui.ls_wideSensor.isChecked(),
            self.ui.ls_swap_left_right.isChecked()
            ))
  def calibrateWhite(self):
    # send white line assumption to robot
    if self.robot.info.talkToBridge:
      self.robot.conWrite("robot licw\n")
    else:
      self.robot.conWrite("licw\n")
  def calibrateBlack(self):
    # send white line assumption to robot
    if self.robot.info.talkToBridge:
      self.robot.conWrite("robot licb\n")
    else:
      self.robot.conWrite("licb\n")
  #def stepOrVelValueChanged(self):
    #pass # save step and base vel in right config
    #if (not self.inUpdate):
      #if (self.ui.reg_vel_use.isChecked()):
        #self.velBaseMotorReg = self.ui.reg_turn_step_vel.value()
      #else:
        #self.velBaseMotorV = self.ui.reg_turn_step_vel.value()
      ## step size depend on turn and vel regulator use
      #if (self.ui.reg_turn_use.isChecked()):
        #self.stepValueTurnReg = self.ui.reg_turn_step_val.value()
      #else:
        #if (self.ui.reg_vel_use.isChecked()):
          #self.stepValueMotorReg = self.ui.reg_turn_step_val.value()
        #else:
          #self.stepValueMotorV = self.ui.reg_turn_step_val.value()
  def butEdit(self):
    if (not self.robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def butCancel(self):
    # cancel button pressed
    self.inEdit = False
  #
  def butApply(self):
    self.setWhiteLine()
    self.inEdit = False
    
  def justConnected(self):
    self.lastTab = "none"

