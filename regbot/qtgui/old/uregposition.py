#!/usr/bin/python
# -*- coding: utf-8 -*-

import threading
#import numpy as np
#import pyqtgraph as pg




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

  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui
    self.parent = robot.parent

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
    if (not self.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      self.ui.reg_pos_apply.setEnabled(False)
      self.ui.reg_pos_start.setEnabled(False)
      self.ui.reg_pos_cancel.setEnabled(False)
      self.ui.reg_pos_edit.setEnabled(True)
    else:
      self.ui.reg_pos_apply.setEnabled(self.inEdit)
      self.ui.reg_pos_cancel.setEnabled(self.inEdit)
      self.ui.reg_pos_edit.setEnabled(not self.inEdit)
      self.ui.reg_pos_start.setEnabled(not self.inEdit)
    if (self.dataRead):
      self.dataRead = False
      self.inUpdate = True
      if (not self.inEdit):
        pass # reset with robot data
        # base speed depend on velocity regulator
        self.ui.reg_pos_use.setChecked(self.regActive)
        self.ui.reg_pos_KP.setValue(self.regKp)
        self.ui.reg_pos_tau_d.setValue(self.regTauD)
        self.ui.reg_pos_tau_i.setValue(self.regTauI)
        self.ui.reg_pos_alpha.setValue(self.regAlpha)
        self.ui.reg_pos_ilimit.setValue(self.regIMax)
        self.ui.reg_pos_out_limit.setValue(self.regOutMax)
        self.ui.reg_pos_step_time.setValue(self.regStepTime)
        self.ui.reg_pos_step_from.setValue(self.regStepFrom)
        self.ui.reg_pos_step_to.setValue(self.regStepTo)
        self.ui.reg_pos_LeadFwd.setChecked(self.regTurnLeadFwd)
      self.inUpdate = False
      if (self.dataReadZ):
        self.dataReadZ = False
        if (self.regulZNumer[1] == 0.0):
          self.ui.reg_pos_numer.setText("%g" % (self.regulZNumer[0]))
          self.ui.reg_pos_denom.setText("1")
        else:
          self.ui.reg_pos_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
          self.ui.reg_pos_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
      if (self.dataReadZI):
        self.dataReadZI = False
        if (self.regulZINumer[1] == 0.0):
          self.ui.reg_pos_numer_2.setText("0")
          self.ui.reg_pos_denom_2.setText("1")
        else:
          self.ui.reg_pos_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
          self.ui.reg_pos_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not self.robot.timerUpdate):
      #print("turn data changed manuallt - not in timed update")
      self.inEdit = True
  def configChanged(self):
    self.inUpdate = True
    pass
    self.inUpdate = False
  def regulatorUseClicked(self):
    self.configChanged();
    self.parent.pos_paint_space.repaint()
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
    self.robot.conWrite("M=7\n")
    self.robot.conWrite("start\n")
    self.robot.mainStatus += " mission 7 started\n"
    self.robot.mainStatusSet = True
  def regulatorParamApply(self):
    self.robot.conWrite("rp=%d %g %g %g %g %g %g %g %g %d %g\n" % (
        self.ui.reg_pos_use.isChecked(), 
        self.ui.reg_pos_KP.value(), 
        self.ui.reg_pos_tau_i.value(), 
        self.ui.reg_pos_tau_d.value(), 
        self.ui.reg_pos_alpha.value(),
        self.ui.reg_pos_ilimit.value(),
        self.ui.reg_pos_step_time.value(),
        self.ui.reg_pos_step_from.value(),
        self.ui.reg_pos_step_to.value(),
        self.ui.reg_pos_LeadFwd.isChecked(),
        self.ui.reg_pos_out_limit.value()
        ))
    #self.ui.reg_turn_apply.setEnabled(False)
    #self.regTurn.stepOrVelValueChanged()
    self.inEdit = False
