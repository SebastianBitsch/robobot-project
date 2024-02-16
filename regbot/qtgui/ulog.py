#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Data logging functions
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
import time

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
  log_ctrl_vel = 0;
  log_ctrl_turn = 0;
  log_ctrl_pos = 0;
  log_ctrl_edge = 0;
  log_ctrl_wall = 0;
  log_ctrl_fwd_dist = 0;
  log_ctrl_bal = 0;
  log_ctrl_bal_vel = 0;
  log_ctrl_bal_pos = 0;
  #log_lbo = 0 # barometer not used
  #log_lbc = 0 # log of balance control details
  log_lct = 0 # control time in us
  log_lin = 0 # interval
  sampleTime = 0.001 # REGBOT sample time - normally 1ms
  log_ltr = 0; # turn rate
  log_lcn = [0,0] # log buffer rows and max rows
  log_lex = 0
  log_set_manually = False
  logFlagRead = False
  logDataRead = False
  lock = threading.RLock()
  logList = ""
  logData = ""
  lastDataRequestTime = time.time()
  lastTab = ""

  #
  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui

  def timerUpdate(self):
    if self.logFlagRead:
      self.ui.log_buf_cnt.setText("used " + str(self.log_lcn[0]) + "/" + str(self.log_lcn[1]))
      self.ui.log_buffer_time.setText("Log time " + str(self.log_lin * self.log_lcn[1] * self.sampleTime) + " sec")
      #if not self.log_set_manually:
      self.lock.acquire()
      self.ui.log_lms.setChecked(self.log_lms)
      self.ui.log_lac.setChecked(self.log_lac)
      self.ui.log_lgy.setChecked(self.log_lgy)
      self.ui.log_lma.setChecked(self.log_lma)
      self.ui.log_lvr.setChecked(self.log_lvr)
      self.ui.log_lmv.setChecked(self.log_lmv)
      self.ui.log_lmr.setChecked(self.log_lmr)
      self.ui.log_lme.setChecked(self.log_lme)
      self.ui.log_turn_rate.setChecked(self.log_ltr)
      self.ui.log_lpo.setChecked(self.log_lpo)
      self.ui.log_line.setChecked(self.log_line)
      self.ui.log_distance.setChecked(self.log_dist)
      self.ui.log_lbt.setChecked(self.log_lbt)
      #self.ui.log_lbc.setChecked(self.log_lbc)
      self.ui.log_lct.setChecked(self.log_lct)
      self.ui.log_lex.setChecked(self.log_lex)
      #
      self.ui.log_ctrl_vel.setChecked(self.log_ctrl_vel)
      self.ui.log_ctrl_turn.setChecked(self.log_ctrl_turn)
      self.ui.log_ctrl_pos.setChecked(self.log_ctrl_pos)
      self.ui.log_ctrl_edge.setChecked(self.log_ctrl_edge)
      self.ui.log_ctrl_wall.setChecked(self.log_ctrl_wall)
      self.ui.log_ctrl_fwd_dist.setChecked(self.log_ctrl_fwd_dist)
      self.ui.log_ctrl_bal.setChecked(self.log_ctrl_bal)
      self.ui.log_ctrl_bal_vel.setChecked(self.log_ctrl_bal_vel)
      self.ui.log_ctrl_bal_pos.setChecked(self.log_ctrl_bal_pos)
      #
      self.ui.log_interval.setValue(self.log_lin)
      #self.ui.log_allow.setChecked(self.log_allow)
      #self.ui.log_lbo.setChecked(self.log_lbo)
      self.lock.release()
    if (self.logDataRead):
      self.lock.acquire()
      self.ui.log_view.setText(self.logList + self.logData)
      self.logDataRead = False
      self.lock.release()
    # request update at times - if changed by another client
    if (self.lastTab != self.robot.currentTab):
      self.lastTab = self.robot.currentTab;
      if (self.robot.info.talkToBridge and self.robot.currentTab == "log"):
        if (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected():
          ##  subscribe to base data IS, version and mission status
          self.robot.conWrite("lms subscribe 6\n")
          self.robot.conWrite("lac subscribe 6\n")
          self.robot.conWrite("lgy subscribe 6\n")
          self.robot.conWrite("lvr subscribe 6\n")
          self.robot.conWrite("lmv subscribe 6\n")
          self.robot.conWrite("lma subscribe 6\n")
          self.robot.conWrite("lmr subscribe 6\n")
          self.robot.conWrite("ltr subscribe 6\n")
          self.robot.conWrite("lme subscribe 6\n")
          self.robot.conWrite("lpo subscribe 6\n")
          self.robot.conWrite("line subscribe 6\n")
          self.robot.conWrite("ldi subscribe 6\n")
          self.robot.conWrite("lbt subscribe 6\n")
          self.robot.conWrite("lex subscribe 6\n")
          self.robot.conWrite("lin subscribe 6\n")
          self.robot.conWrite("lct subscribe 6\n")
          self.robot.conWrite("lcl subscribe 6\n") # controller flags
          self.robot.conWrite("lcn subscribe 6\n") # log entry count
          print("LOG.PY: subscribed to all log flags\n")
        # no need to unsubscribe
    if self.robot.currentTab == "log" and not self.robot.info.talkToBridge:
      if time.time() - self.lastDataRequestTime > 1.5:
        self.robot.conWrite("u3\n")
        self.lastDataRequestTime = time.time()


  #
  def readData(self, gg, line):
    dataUsed = True
    self.lock.acquire()
    try:
      if (gg[0][0] == 'l'):
        if (gg[0] == 'logdata'):
          # log data from bridge - direct logdata is handled at the end
          # skip the keyword, just add the data
          logData = line[8:]
          if (gg[1][0] == '%'):
            self.logList += logData
          if ((gg[1][0] >= '0' and gg[1][0] <= '9') or gg[1][0] == '.'):
            self.logData += logData
          self.logDataRead = True
        # else one of the log-flags
        elif gg[0] == "lms":
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
        #elif gg[0] == "lbc":
          #self.log_lbc = int(gg[1],0)
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
          if (len(gg) > 3):
            self.sampleTime = float(gg[3])
        elif gg[0] == "lcl":
          try:
            self.log_ctrl_vel = int(gg[1],0)
            self.log_ctrl_turn = int(gg[2],0)
            self.log_ctrl_pos = int(gg[3],0)
            self.log_ctrl_edge = int(gg[4],0)
            self.log_ctrl_wall = int(gg[5],0)
            self.log_ctrl_fwd_dist = int(gg[6],0)
            self.log_ctrl_bal = int(gg[7],0)
            self.log_ctrl_bal_vel = int(gg[8],0)
            self.log_ctrl_bal_pos = int(gg[9],0)
          except:
            print("too few ctrl log parameters")
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
  #def logSetManually(self):
    #if (not self.log_set_manually):
      #self.log_set_manually = True
      #self.ui.log_flag_apply.setEnabled(True)
  def logSave(self):
    try:
      f = open(self.ui.log_filename.text(), "w")
      if (self.ui.log_save_header.isChecked()):
        f.write('%% logfile from robot ' + self.ui.robot_id_main.text() + '\n')
        f.write(self.logList)
      if (self.ui.log_save_config.isChecked()):
        f.write('%% Configuration as seen in client (assumed updated from robot)\n')
        fil = open('.regbotConfigTemp.ini', 'r');
        for line in fil:
          f.write('% ' + line)
        fil.close()
        f.write('%% data log\n')
      f.write(self.logData)
      f.close()
    except:
      self.ui.statusbar.showMessage("Failed to open file " + self.ui.log_filename.text() + "!", 3000)
  def logClear(self):
    self.logData = ""
    self.logList = ""
    self.logDataRead = True

  def justConnected(self):
    self.lastTab = "none"
