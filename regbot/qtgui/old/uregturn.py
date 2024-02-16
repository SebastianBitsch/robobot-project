#!/usr/bin/python
# -*- coding: utf-8 -*-

import threading
#import numpy as np
#import pyqtgraph as pg



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

  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui
    self.parent = robot.parent

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
    if (not self.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      self.ui.reg_turn_apply.setEnabled(False)
      self.ui.reg_turn_start.setEnabled(False)
      self.ui.reg_turn_read.setEnabled(False)
      self.ui.reg_turn_edit.setEnabled(True)
    else:
      self.ui.reg_turn_apply.setEnabled(self.inEdit)
      self.ui.reg_turn_read.setEnabled(self.inEdit)
      self.ui.reg_turn_edit.setEnabled(not self.inEdit)
      self.ui.reg_turn_start.setEnabled(not self.inEdit)
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
        if (self.ui.reg_turn_use.isChecked()):
          self.stepValueTurnReg = self.regStepVal
        else:
          if (self.ui.reg_vel_use.isChecked()):
            self.stepValueMotorReg = self.regStepVal
          else:
            self.stepValueMotorV = self.regStepVal
        self.ui.reg_turn_step_val.setValue(self.regStepVal)
        self.ui.reg_turn_use.setChecked(self.regActive)
        self.ui.reg_turn_KP.setValue(self.regKp)
        self.ui.reg_turn_tau_d.setValue(self.regTauD)
        self.ui.reg_turn_tau_i.setValue(self.regTauI)
        self.ui.reg_turn_alpha.setValue(self.regAlpha)
        self.ui.reg_turn_ilimit.setValue(self.regIMax)
        self.ui.reg_turn_out_limit.setValue(self.regOutMax)
        self.ui.reg_turn_step_on.setValue(self.regStepOn)
        self.ui.reg_turn_step_off.setValue(self.regStepOff)
        self.ui.reg_turn_step_vel.setValue(self.regStepVel)
        self.ui.reg_turn_LeadFwd.setChecked(self.regTurnLeadFwd)
      self.inUpdate = False
      if (self.dataReadZ):
        self.dataReadZ = False
        if (self.regulZNumer[1] == 0.0):
          self.ui.reg_turn_numer.setText("%g" % (self.regulZNumer[0]))
          self.ui.reg_turn_denom.setText("1")
        else:
          self.ui.reg_turn_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
          self.ui.reg_turn_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
      if (self.dataReadZI):
        self.dataReadZI = False
        if (self.regulZINumer[1] == 0.0):
          self.ui.reg_turn_numer_2.setText("0")
          self.ui.reg_turn_denom_2.setText("1")
        else:
          self.ui.reg_turn_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
          self.ui.reg_turn_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not self.robot.timerUpdate):
      #print("turn data changed manuallt - no timer")
      self.inEdit = True
  def configChanged(self):
    self.inUpdate = True
    if (self.ui.reg_vel_use.isChecked()):
      self.ui.reg_turn_step_vel.setValue(self.velBaseMotorReg)
    else:
      self.ui.reg_turn_step_vel.setValue(self.velBaseMotorV)
    # step size depend on turn and vel regulator use
    if (self.ui.reg_turn_use.isChecked()):
      self.ui.reg_turn_step_val.setValue(self.stepValueTurnReg)
      self.ui.reg_turn_step_value_label.setText("Value [rad]")
    else:
      if (self.ui.reg_vel_use.isChecked()):
        self.ui.reg_turn_step_val.setValue(self.stepValueMotorReg)
        self.ui.reg_turn_step_value_label.setText("Value [rad/s]")
      else:
        self.ui.reg_turn_step_val.setValue(self.stepValueMotorV)
        self.ui.reg_turn_step_value_label.setText("Value [V]")
    self.inUpdate = False
  def regulatorUseClicked(self):
    self.configChanged();
    self.parent.turn_paint_space.repaint()
  def regulatorParamRead(self):
    # cancel pressed
    self.inEdit = False
  def stepOrVelValueChanged(self):
    pass # save step and base vel in right config
    if (not self.inUpdate):
      if (self.ui.reg_vel_use.isChecked()):
        self.velBaseMotorReg = self.ui.reg_turn_step_vel.value()
      else:
        self.velBaseMotorV = self.ui.reg_turn_step_vel.value()
      # step size depend on turn and vel regulator use
      if (self.ui.reg_turn_use.isChecked()):
        self.stepValueTurnReg = self.ui.reg_turn_step_val.value()
      else:
        if (self.ui.reg_vel_use.isChecked()):
          self.stepValueMotorReg = self.ui.reg_turn_step_val.value()
        else:
          self.stepValueMotorV = self.ui.reg_turn_step_val.value()
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
