#!/usr/bin/python
# -*- coding: utf-8 -*-

import threading
#import numpy as np
#import pyqtgraph as pg




#class URegVel(object):
  #regActive = 0
  #regKp = 0.0
  #regTauD = 0.0
  #regTauI = 0.0
  #regAlpha = 0.0
  #regIMax = 0.0
  #regStepTime = 0.0
  #regStepFrom = 0.0
  #regStepTo = 0.0
  #dataRead = False
  #dataReadZ = False
  #dataReadZI = False
  #regulZNumer = [1.0, 0.0]
  #regulZDenom = [0.0, 0.0]
  #regulZINumer = [1.0, 0.0]
  #regulZIDenom = [0.0, 0.0]
  #regVelAccLimit = 100
  #regVelEstUse = True
  #regVelLeadFwd = True
  #regVelVoltLimit = 10 # motor voltage limit [V]
  ## if voltage control
  #regStepFromV = 2.1
  #regStepToV = 4.1
  ## if m/s control
  #regStepFromReg = 0.20
  #regStepToReg = 0.40
  #inUpdate = False
  #inEdit = False
  ##
  #about_box= None
  ## resource lock
  #lock = threading.RLock()

  #def __init__(self, robot):
    #self.robot = robot
    #self.ui = robot.ui

  #def readData(self, gg):
    #used = True
    #self.lock.acquire()
    ##print("URegVel " + gg[0] + " length is " + str(len(gg)))
    #try: 
      #if gg[0] == "rgv":
        #self.regActive = int(gg[1],0)
        #self.dataRead = True
        #self.regKp = float(gg[2])
        #self.regTauI = float(gg[3])
        #self.regTauD = float(gg[4])
        #self.regAlpha = float(gg[5])
        #self.regIMax = float(gg[6])
        #self.regStepTime = float(gg[7])
        #self.regStepFrom = float(gg[8])
        #self.regStepTo = float(gg[9])
        #if (len(gg) > 10):
          #self.regVelAccLimit = float(gg[10])
        #if (len(gg) > 11):
          #self.regVelEstUse = int(gg[11])
        #if (len(gg) > 12):
          #self.regVelLeadFwd = int(gg[12])
        #if (len(gg) > 13):
          #self.regVelVoltLimit = float(gg[13]) 
      #elif gg[0] == "rgvz":
        #self.regulZDenom[1] = float(gg[1])
        ## self.regulZDenom[2] = float(gg[2])
        #self.regulZNumer[0] = float(gg[2])
        #self.regulZNumer[1] = float(gg[3])
        ## self.regulZNumer[2] = float(gg[5])
        #self.dataReadZ = True
      #elif gg[0] == "rgvzi":
        #self.regulZIDenom[1] = float(gg[1])
        #self.regulZINumer[0] = float(gg[2])
        #self.regulZINumer[1] = float(gg[3])
        #self.dataReadZI = True
      #else:
        #used = False
    #except:
      #print("URegVel: data read error - skipped a " + gg[0])
      #pass
    #self.lock.release()
    #return used
  #def showData(self):
    #self.lock.acquire()
    #if (not self.ui.frame_batt_time.isEnabled()):
      ## no live data from robot
      #self.ui.reg_vel_apply.setEnabled(False)
      #self.ui.reg_vel_start.setEnabled(False)
      #self.ui.reg_vel_read.setEnabled(False)
    #elif self.inEdit:
      #self.ui.reg_vel_apply.setEnabled(True)
      #self.ui.reg_vel_read.setEnabled(True)
    #else:
      #self.ui.reg_vel_start.setEnabled(True)
    #if (self.dataRead):
      #self.dataRead = False
      #self.inUpdate = True
      ## base speed depend on velocity regulator
      #if (not self.inEdit):
        #if (self.regActive):
          #self.regStepFromReg = self.regStepFrom
          #self.regStepToReg = self.regStepTo
        #else:
          #self.regStepFromV = self.regStepFrom
          #self.regStepToV = self.regStepTo
        ## reset with robot data
        #self.ui.reg_vel_use.setChecked(self.regActive)
        #if (self.regActive):
          #self.ui.reg_turn_vel_base.setText("base [m/s]")
        #else:
          #self.ui.reg_turn_vel_base.setText("base [V]")
        #self.ui.reg_vel_KP.setValue(self.regKp)
        #self.ui.reg_vel_tau_d.setValue(self.regTauD)
        #self.ui.reg_vel_tau_i.setValue(self.regTauI)
        #self.ui.reg_vel_alpha.setValue(self.regAlpha)
        #self.ui.reg_vel_integrate_max.setValue(self.regIMax)
        #self.ui.reg_vel_apply.setEnabled(False)
        #self.ui.reg_vel_start.setEnabled(True)
        #self.ui.reg_vel_edit.setEnabled(True)
        #self.ui.reg_vel_read.setEnabled(False)
        #self.ui.reg_vel_steptime.setValue(self.regStepTime)
        #self.ui.reg_vel_step_from.setValue(self.regStepFrom)
        #self.ui.reg_vel_step_to.setValue(self.regStepTo)
        #if (self.regVelAccLimit < 99):
          #self.ui.vel_acc_limit_use.setChecked(True)
          #self.ui.reg_vel_acc_limit.setValue(self.regVelAccLimit)
        #else:
          #self.ui.vel_acc_limit_use.setChecked(False)
          #self.ui.reg_vel_acc_limit.setValue(100)
        #self.ui.reg_vel_est_use.setChecked(self.regVelEstUse)
        #self.ui.reg_vel_LeadFwd.setChecked(self.regVelLeadFwd)
        #self.ui.reg_vel_volt_limit.setValue(self.regVelVoltLimit)
      #self.inUpdate = False
    #if (self.dataReadZ):
      #self.dataReadZ = False
      #if (self.regulZNumer[1] == 0.0):
        #self.ui.reg_vel_numer.setText("%g" % (self.regulZNumer[0]))
        #self.ui.reg_vel_denom.setText("1")
      #else:
        #self.ui.reg_vel_numer.setText("%g  -  %g z^-1" % (self.regulZNumer[0], -self.regulZNumer[1]))
        #self.ui.reg_vel_denom.setText( "1  -  %g z^-1" % (-self.regulZDenom[1]))
    #if (self.dataReadZI):
      #self.dataReadZI = False
      #if (self.regulZINumer[1] == 0.0):
        #self.ui.reg_vel_numer_2.setText("0")
        #self.ui.reg_vel_denom_2.setText("1")
      #else:
        #self.ui.reg_vel_numer_2.setText("%g  +  %g z^-1" % (self.regulZINumer[0], self.regulZINumer[1]))
        #self.ui.reg_vel_denom_2.setText( "1  -  %g z^-1" % (-self.regulZIDenom[1]))
    #self.lock.release()
  ## when any of the parameters are changed - allow apply button to be pressed
  #def dataChangedManually(self):
    #if (not self.robot.timerUpdate):
      #self.lock.acquire()
      #self.inEdit = True
      #self.ui.reg_vel_apply.setEnabled(True)
      #self.ui.reg_vel_start.setEnabled(False)
      #self.ui.reg_vel_edit.setEnabled(False)
      #self.ui.reg_vel_read.setEnabled(True)
      #self.lock.release()
  #def configChanged(self):
    #pass # load the right value into step from-to widget
    #self.inUpdate = True
    #if (self.ui.reg_vel_use.isChecked()):
      #self.ui.reg_vel_step_from.setValue(self.regStepFromReg)
      #self.ui.reg_vel_step_to.setValue(self.regStepToReg)
      #self.ui.reg_turn_vel_base.setText("base [m/s]")
      #self.ui.reg_vel_step_label.setText("From [m/s]")
      #self.ui.reg_vel_step_label_2.setText("To [m/s]")
    #else:
      #self.ui.reg_vel_step_from.setValue(self.regStepFromV)
      #self.ui.reg_vel_step_to.setValue(self.regStepToV)      
      #self.ui.reg_turn_vel_base.setText("base [V]")
      #self.ui.reg_vel_step_label.setText("From [Volt]")
      #self.ui.reg_vel_step_label_2.setText("To [Volt]")
    #self.inUpdate = False
  #def regulatorUseClicked(self):
    #self.robot.regTurn.configChanged()
    #self.configChanged()
    #self.ui.reg_vel_frame.repaint()
  #def regulatorParamRead(self):
    ## this is the cancel button
    ## just disable edit, then data is from robot
    ## self.ui.reg_vel_apply.setEnabled(False)
    #self.inEdit = False
  #def stepValueChanged(self):
    #pass # save step and base vel in right config
    #if (not self.inUpdate):
      #if (self.ui.reg_vel_use.isChecked()):
        #self.regStepFromReg = self.ui.reg_vel_step_from.value()
        #self.regStepToReg = self.ui.reg_vel_step_to.value()
      #else:
        #self.regStepFromV = self.ui.reg_vel_step_from.value()
        #self.regStepToV = self.ui.reg_vel_step_to.value()
    ##
  #def accLimitUse(self):
    #self.inUpdate = True
    #if (not self.ui.vel_acc_limit_use.isChecked()):
      #self.ui.reg_vel_acc_limit.setValue(100.0)
    #else:    
      #self.ui.reg_vel_acc_limit.setValue(self.regVelAccLimit)
    #self.inUpdate = False
  #def helpbox(self):
    ## QMessageBox.about (QWidget parent, QString caption, QString text)
    #if (self.about_box == None):
      #self.about_box = QtGui.QMessageBox(mymw)
      #self.about_box.setText('''<p><span style=" font-size:20pt;">
                #Motor velocity control</span></p>
                #<p>
                #<b>MOTOR VEL CONTROL</b> (both motors): <br />
                #<i>USE not ticked</i>:<br />
                #* Velocity is open loop (STEP is in Volts)<br />
                #<i>USE is ticked</i>:<br />
                #* Velocity controller is in closed loop (ref is in m/s)<br />
                #* <b>Kp</b> is proportional gain<br />
                #* <b>TAU_I</b> is entered in seconds.<br />
                #* if TAU_I=0, then the I-term is omitted.<br />
                #The lead term is either in the forward branch or in the reverse branch, controllerd by <b>Lead in forward</b>.<br />
                #* <b>TAU_D</b> is the lead time constant [sec] and works together with <b>alpha</b>.<br />
                #* if TAU_D=0 then the Lead term is omitted.<br />
                #* <b>I-MAX</b> limits the integration term [V] symmetric +/-, if 0 then no limit.</p>
                #<hr />
                #<p><b>STEP</b> is the step applied when START is pressed <br />
                #open loop:<br />
                #* the input <b>STEP FROM</b> and <b>STEP TO</b> is in motor anchor voltage [V]<br />
                #voltages below 1.0 volt will in general not start the motors.<br />
                #maximum voltage is +/-9V<br />
                #closed loop:<br />
                #* velocity is closed loop, with a controller as specified by the K_P etc.<br />
                #* The STEP FROM and STEP TO is now in rad/s. minimum is in the order of +/- 0.06 m/s, maximum is about +/-1.5 m/s </p>
                #<hr />
                #<p><b>Edit</b> will stop updating of fieds from robot<br/>
                #When <b>Cancel</b> is pressed settings are copied from the robot<br />
                #When <b>APPLY</b> is pressed the new settings are implemented on the robot. 
                #When <b>START</b> is pressed, then the step (mission 0) is started.
                #(Enabled when shown values are applied to the robot).</p>
                #<hr />
                #<p><b>Other settings</b><br />
                #The <b>acceleration limit</b> limits the acceleration by limiting the increase rate in the velocity reference.<br /> 
                #When using balance control this acceleration limit is used for the mission velocity.<br />
                #The <b>velocity estimator</b> uses the change in anchor voltage to estimate a velocity change and merges 
                #this with the encoder velocity estimate in a complementary filter, where the crossover frequency depends on the actual velocity.
                #This has no effect if the velocity is above ~10cm/sec.<br />
                #<b>Motor voltage limit</b> limits the maximum voltage the motor will ever see (limits motor PWM).
                #</p>''');
      ##about_box.setIconPixmap(QtGui.QPixmap("dtulogo_125x182.png"))
      #self.about_box.setWindowTitle("regbot motor velocity")
      #self.about_box.setWindowModality(QtCore.Qt.NonModal)
    #self.about_box.show()
