#!/usr/bin/python
# -*- coding: utf-8 -*-

import threading
#import numpy as np
#import pyqtgraph as pg



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

  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui

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
    if (not self.ui.frame_batt_time.isEnabled()):
      # no live data from robot
      self.ui.reg_bal_apply.setEnabled(False)
      self.ui.reg_bal_start.setEnabled(False)
      self.ui.reg_bal_read.setEnabled(False) # cancel
      self.ui.reg_bal_edit.setEnabled(not self.inEdit)
    else:
      self.ui.reg_bal_start.setEnabled(not self.inEdit)
      self.ui.reg_bal_apply.setEnabled(self.inEdit)
      self.ui.reg_bal_read.setEnabled(self.inEdit) # cancel
      self.ui.reg_bal_edit.setEnabled(not self.inEdit)
    self.ui.reg_bal_LeadFwd.setEnabled(not self.ui.reg_bal_LeadGyro.isChecked())
    if (self.dataRead and self.dataReadMV):
      self.dataRead = False
      self.dataReadMV = False
      self.inUpdate = True
      # base speed depend on velocity regulator
      if (not self.inEdit):
        # reset with robot data
        self.ui.reg_bal_use.setChecked(self.regActive)
        self.ui.reg_bal_KP.setValue(self.regKp)
        self.ui.reg_bal_tau_d.setValue(self.regTauD)
        self.ui.reg_bal_tau_i.setValue(self.regTauI)
        self.ui.reg_bal_alpha.setValue(self.regAlpha)
        self.ui.reg_bal_integrate_max.setValue(self.regIMax)
        self.ui.reg_bal_out_limit.setValue(self.regOutMax)
        if (self.regOutMax < 0.5):
          self.ui.label_out_limit.setStyleSheet("QLabel { background-color : red; }")
        else:
          self.ui.label_out_limit.setStyleSheet("QLabel { background-color : lightGray; }")
        self.ui.reg_bal_steptime.setValue(self.regStepTime)
        self.ui.reg_bal_LeadFwd.setChecked(self.regLeadFwd)
        self.ui.reg_bal_LeadGyro.setChecked(self.regLeadGyro)
        self.ui.reg_bal_alpha.setEnabled(not self.regLeadGyro)
        #print("show data bal V:" + str(self.regStepVFrom) + "-" + str(self.regStepVTo) + ", B: " + str(self.regStepFrom) + "-" + str(self.regStepTo))
        if (self.regMvUse):
          self.ui.reg_bal_step_from.setValue(self.regStepVFrom)
          self.ui.reg_bal_step_to.setValue(self.regStepVTo)
        else:
          self.ui.reg_bal_step_from.setValue(self.regStepFrom)
          self.ui.reg_bal_step_to.setValue(self.regStepTo)
        # mission velocity ctrl
        self.ui.reg_mvel_KP.setValue(self.regMvKp)
        self.ui.reg_mvel_tau_d.setValue(self.regMvTauD)
        self.ui.reg_mvel_tau_i.setValue(self.regMvTauI)
        self.ui.reg_mvel_alpha.setValue(self.regMvAlpha)
        self.ui.reg_mvel_zeta.setValue(self.regMvZeta)
        self.ui.reg_mvel_integrate_max.setValue(self.regMvIMax)
        self.ui.reg_balvel_use.setChecked(self.regMvUse)
        self.ui.reg_mvel_LeadFwd.setChecked(self.regMvLeadFwd)
        self.ui.reg_mvel_out_limit.setValue(self.regMvOutMax)
      self.inUpdate = False
    if (self.dataReadZ):
      self.dataReadZ = False
      if (self.regulZNumer[1] == 0.0):
        self.ui.reg_bal_numer.setText("%g" % (self.regulZNumer[0]))
        self.ui.reg_bal_denom.setText("1")
      else:
        self.ui.reg_bal_numer.setText("%g + %g z^-1" % (self.regulZNumer[0], self.regulZNumer[1]))
        self.ui.reg_bal_denom.setText( "1 - %g z^-1" % (-self.regulZDenom[1]))
    if (self.dataReadZI):
      self.dataReadZI = False
      if (self.regulZINumer[1] == 0.0):
        self.ui.reg_bali_numer.setText("0")
        self.ui.reg_bali_denom.setText("1")
      else:
        self.ui.reg_bali_numer.setText("%g - %g z^-1" % (self.regulZINumer[0], -self.regulZINumer[1]))
        self.ui.reg_bali_denom.setText( "1 - %g z^-1" % (-self.regulZIDenom[1]))
    if (self.dataReadMvZ):
      self.dataReadMvZ = False
      if (self.regulMvZNumer[1] == 0.0):
        self.ui.reg_bal_mv_numer.setText("%g" % (self.regulMvZNumer[0]))
        self.ui.reg_bal_mv_denom.setText("1")
      else:
        self.ui.reg_bal_mv_numer.setText("%g + %g z^-1 - %g z^-2" % (self.regulMvZNumer[0], self.regulMvZNumer[1], -self.regulMvZNumer[2]))
        self.ui.reg_bal_mv_denom.setText( "1 - %g z^-1 + %g z^-2" % (-self.regulMvZDenom[1], self.regulMvZDenom[2]))
    if (self.dataReadMvZI):
      self.dataReadMvZI = False
      if (self.regulMvZINumer[1] == 0.0):
        self.ui.reg_bal_mv_numer_2.setText("0")
        self.ui.reg_bal_mv_denom_2.setText("1")
      else:
        self.ui.reg_bal_mv_numer_2.setText("%g  -  %g z^-1" % (self.regulMvZINumer[0], -self.regulMvZINumer[1]))
        self.ui.reg_bal_mv_denom_2.setText( "1  -  %g z^-1" % (-self.regulMvZIDenom[1]))
    self.lock.release()
  # when any of the parameters are changed - allow apply button to be pressed
  def dataChangedManually(self):
    if (not self.robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      if ( self.ui.reg_bal_out_limit.value() < 0.5):
        self.ui.label_out_limit.setStyleSheet("QLabel { background-color : red; }")
      else:
        self.ui.label_out_limit.setStyleSheet("QLabel { background-color : lightGray; }")
      if (self.ui.reg_balvel_use.isChecked()):
        self.ui.reg_bal_step_from.setValue(self.regStepVFrom)
        self.ui.reg_bal_step_to.setValue(self.regStepVTo)
      else:
        self.ui.reg_bal_step_from.setValue(self.regStepFrom)
        self.ui.reg_bal_step_to.setValue(self.regStepTo)
      self.lock.release()
  def regulatorUseClicked(self):
    self.configChanged()
    #self.ui.bal_regul_frame.repaint()
  def configChanged(self):
    # load the right value into step from-to widget
    #print("config changed bal V:" + str(self.regStepVFrom) + "-" + str(self.regStepVTo) + ", B: " + str(self.regStepFrom) + "-" + str(self.regStepTo))
    self.inUpdate = True
    if (self.ui.reg_balvel_use.isChecked()):
      self.ui.label_reg_balvel_step_from.setText("step from [m/s]")
      self.ui.label_reg_balvel_step_to.setText("step to [m/s]")
      self.ui.reg_bal_step_from.setValue(self.regStepVFrom)
      self.ui.reg_bal_step_to.setValue(self.regStepVTo)
    elif (self.ui.reg_bal_use.isChecked()):
      self.ui.label_reg_balvel_step_from.setText("step from [rad]")
      self.ui.label_reg_balvel_step_to.setText("step to [rad]")
      self.ui.reg_bal_step_from.setValue(self.regStepFrom)
      self.ui.reg_bal_step_to.setValue(self.regStepTo)
    elif (self.ui.reg_vel_use.isChecked()):
      self.ui.label_reg_balvel_step_from.setText("step from [m/s]")
      self.ui.label_reg_balvel_step_to.setText("step to [m/s]")
      self.ui.reg_bal_step_from.setValue(self.regStepMSFrom)
      self.ui.reg_bal_step_to.setValue(self.regStepMSTo)
    else:
      self.ui.label_reg_balvel_step_from.setText("step from [Volt]")
      self.ui.label_reg_balvel_step_to.setText("step to [Volt]")
      self.ui.reg_bal_step_from.setValue(self.regStepMVFrom)
      self.ui.reg_bal_step_to.setValue(self.regStepMVTo)
    self.inUpdate = False
    self.ui.bal_regul_frame.repaint()
  def regulatorParamRead(self):
    # cancel button pressed
    # just disable the apply buttons, then data is from robot
    self.inEdit = False
    #self.ui.reg_bal_apply.setEnabled(False)
    #self.ui.reg_mvel_apply.setEnabled(False)
  def stepValueChanged(self):
    pass # save step and base vel in right config
    if (not self.inUpdate):
      if (self.ui.reg_balvel_use.isChecked()):
        self.regStepVFrom = self.ui.reg_bal_step_from.value()
        self.regStepVTo = self.ui.reg_bal_step_to.value()
      else:
        self.regStepFrom = self.ui.reg_bal_step_from.value()
        self.regStepTo = self.ui.reg_bal_step_to.value()
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
