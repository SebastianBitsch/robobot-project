
#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2016 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Mission functions
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
from pyqtgraph.Qt import QtGui #, QtCore



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
  gotoDest = 0
  servo = 0
  servoPos = 0
  servoAcc = 0
  chirpUse = False
  chirpVal = 0;
  # continue conditions
  dist = 0
  distUse = '\0'
  velTest = 0
  velTestUse = '\0'
  turn = 0
  turnUse = '\0'
  time = 0
  timeUse = False
  countUse = False
  count = 0
  xingUse = '\0'
  #xingBlackUse = '\0'
  #xingBlackVal = 0;
  xingVal = 0;
  lineValidUse = '\0';
  lineValidVal = 0;  
  lineValidUseRight = '\0';
  lineValidValRight = 0;  
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
  headEndUse = '\0'
  headEnd = 0
  reasonUse = '\0'
  reasonValue = 0
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
    self.eventSetCnt = 0
    self.posUse = False
    self.headUse = False
    # continue conditions
    self.distUse = '\0'
    self.velTestUse = '\0'
    self.turnUse = '\0'
    self.timeUse = False
    self.countUse = False
    self.xingUse = '\0'
    #self.xingBlackUse = '\0'
    self.lineValidUse = '\0'
    #self.lineValidUseRight = '\0'
    self.tiltUse = '\0'
    self.irDist1Use = '\0'
    self.irDist2Use = '\0'
    self.threadUse = False
    self.eventMaskCnt = 0
    self.eventMask = [False] * 5
    self.logFullUse = False
    self.servo = 0
    self.servoPos = 0
    self.servoAcc = 0
    self.headEndUse = '\0'
    self.headEnd = 0
    self.reasonUse = '\0'
    self.reasonValue = 0
    self.chirpUse = False
    self.chirpVal = 0
    
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
      ms = ms + ", head={}".format(self.head)
    if self.eventSetCnt > 0:
      for i in range(0, self.eventSetCnt):
        ms = ms + ", event={:d}".format(self.eventSet[i])
    if (self.servo > 0):
      ms = ms + ", servo={:d}".format(self.servo)
      ms = ms + ", pservo={:d}".format(self.servoPos)
      ms = ms + ", vservo={:d}".format(self.servoAcc)
    if (self.chirpUse):
      ms = ms + ", chirp={}".format(self.chirpVal)
    if (len(ms) > 2):
      # remove first 2 characters
      ms = ms[2:]
    #print("# now ms is " + ms)
    if (self.distUse != '\0'):
      mc = mc + ", dist=" + str(self.dist)
    if (self.velTestUse != '\0'):
      mc = mc + ", vel=" + str(self.velTest)
    if (self.timeUse):
      mc = mc + ", time=" + str(self.time)
    if (self.turnUse != '\0'):
      mc = mc + ", turn=" + str(self.turn)
    if (self.countUse):
      mc = mc + ", count={:d}".format(int(self.count))
    #if (self.xingBlackUse != '\0'):
      #mc = mc + ", xb{}{:d}".format(self.xingBlackUse, int(self.xingBlackVal))
    if (self.xingUse != '\0'):
      mc = mc + ", xl{}{:d}".format(self.xingUse, int(self.xingVal))
    if (self.tiltUse != '\0'):
      mc = mc + ", tilt{}{}".format(self.tiltUse, self.tiltValue)
    if (self.lineValidUse) != '\0':
      mc = mc + ", lv{}{:d}".format(self.lineValidUse, int(self.lineValidVal))
    #if (self.lineValidUseRight) != '\0':
      #mc = mc + ", rv{}{:d}".format(self.lineValidUseRight, int(self.lineValidVal))
    if (self.irDist1Use != '\0'):
      mc = mc + ", ir1{}{}".format(self.irDist1Use, self.irDist1)
    if (self.irDist2Use != '\0'):
      mc = mc + ", ir2{}{}".format(self.irDist2Use, self.irDist2)
    if self.eventMaskCnt > 0:
      for i in range(0,self.eventMaskCnt):
        mc = mc + ", event={:d}".format(self.eventMask[i])
    if self.logFullUse:
      mc = mc + ", log=0"
    if self.headEndUse != '\0':
      mc = mc + ", head{}{}".format(self.headEndUse, self.headEnd)
    if self.reasonUse != '\0':
      mc = mc + ", last{}{:d}".format(self.reasonUse, self.reasonValue)
    if (len(mc) > 2):
      # collect parameter with condition
      ms = ms + ": " + mc[2:]
    #print("#to string end")
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
    # print("# setting mission line from string: " + line)
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
        elif v0 == 'chirp':
          self.chirpVal = float(v1)
          self.chirpUse = True
        elif v0 == 'event':
          iv1 = int(v1)
          if (iv1 >= 0) and (iv1 < 32):
            if self.eventSetCnt >= 5 :
              print("# Error:: mission event cnt exceeds buffer size of 5!  cnt=" + str(self.eventSetCnt))
            else:
              self.eventSet[self.eventSetCnt] = iv1
              self.eventSetCnt = self.eventSetCnt + 1
            # print("# set event mask " + str(self.eventSetCnt) + " value " + str(iv1) + " to true")
          else:
            error = True
            break
        elif v0 == 'servo':
          self.servo = int(v1)
        elif v0 == 'pservo':
          self.servoPos = int(v1)
        elif v0 == 'vservo':
          self.servoAcc = int(v1)
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
            if len(val) != 2: # not a =
              valc = '<'
              val = prepart[j].split(valc)
            if len(val) != 2: # not a <
              valc = '>'
              val = prepart[j].split(valc)
            if len(val) != 2: # not a >
              valc = '!'
              val = prepart[j].split(valc)
            if len(val) != 2: # not a !
              error = True # no other (valid) possibility
              print("# PY: invalid operator in: " + str(prepart[j]))
              break
            v0 = val[0].strip().lower()
            v1 = val[1].strip()
            error = not self.isFloat(v1)
            if (error):
              break
            if v0 == "dist":
              self.dist = float(v1)
              self.distUse = valc
            elif v0 == "vel":
              self.velTest = float(v1)
              self.velTestUse = valc
            elif v0 == "turn":
              self.turn = float(v1)
              self.turnUse = valc
            elif v0 == "time":
              self.timeUse = valc
              self.time = float(v1)
            elif v0 == "count":
              self.countUse = valc
              self.count = float(v1)
            #elif v0 == "xb":
              #self.xingBlackUse = valc
              #self.xingBlackVal = int(v1)
            elif v0 == "xl":
              #print("#got crossing white line")
              self.xingUse = valc
              self.xingVal = int(v1)
            elif v0 == "lv":
              #print("#got line valid line")
              self.lineValidUse = valc
              self.lineValidVal = int(v1)
            #elif v0 == "rv":
              ##print("#got line valid line")
              #self.lineValidUseRight = valc
              #self.lineValidValRight = int(v1)
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
                if (self.eventMaskCnt < 4):
                  self.eventMaskCnt = self.eventMaskCnt + 1
                else:
                  print("Mission line has too many event statements? " + str(self.eventMaskCnt) + ">=5 - ignored")
                #print("# set event mask " + str(self.eventMaskCnt) + " value " + str(iv1) + " to true")
              else:
                error = True
                break
            elif v0 == "log":
              self.logFullUse = True;
            elif v0 == "head":
              self.headEndUse = valc
              self.headEnd = float(v1)
            elif v0 == "last":
              self.reasonUse = valc
              self.reasonValue = int(v1)
            else:
              error = True
              break;
      #else:
        ## there should always be a continue condition
        #error = True;
    if error:
      print("# error in line")
      errstr = " near '" + prepart[j] + "'!"
      #if (len(val) >= 2):
        #errstr = errstr + " val[0]='" + v0 + "' val[1]='" + val[1] + "' + len(val)=" + str(len(val))
      self.valid = False
    else:
      self.valid = self.accUse or self.balUse or self.distUse != '\0' or self.velTestUse != '\0' \
        or self.logUse or self.timeUse or self.trUse or self.turnUse != '\0' or self.velUse \
        or self.lineLUse or self.lineRUse or self.posUse or self.headUse \
        or self.lineWhiteUse or self.chirpUse \
        or self.xingUse != '\0' or self.lineValidUse != '\0'  or self.tiltUse != '\0' \
        or self.irDistUse or self.irSensorUse or self.irDist1Use != '\0' or self.irDist2Use != '\0' \
        or self.gotoUse or self.eventSetCnt > 0 or self.eventMaskCnt > 0 or self.logFullUse
    #print("# PY setting mission line from string end (err:" + str(error) + "): " + line)
    #print("# PY seen line as (err:" + str(error) + "): " + self.toString())
    return errstr
  #
  # end of UMissionLine


class UMission(object):
  startSwitch = False
  # current mission to run
  mission = -1
  missionState = 0
  missionLineState = 0
  missionThreadState = 0
  missionDataRead = True
  missionName = "mission name"
  missionManually = False
  lock = threading.RLock()
  #mission_directory = ""
  #mission_filename = "regb0t_mission.mis"
  statusWhileRunning = 0;
  missionTextMsg = ""
  missionTextMsgChaged = True
  missionTextEdit = ""
  missionTextEditChaged = True
  # user mission lines
  mLines = []
  mLinesNewData = False
  about_box = None
  checkNr = 0
  lastFileName = ('my_mission.mis','mission (*.mis)')
  lastTab = "notme"
  
  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui
  
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
  def timerUpdate(self):
    if (self.robot.info.talkToBridge):
      if (self.robot.wifiConnected and not self.robot.wifiWaiting4reply) or self.robot.isConnected():
        if (self.lastTab != self.robot.currentTab and self.robot.currentTab == "mission"):
          self.lastTab = self.robot.currentTab
          # subscribe to relevant items - mission lines
          self.robot.conWrite("<m subscribe 6\n")
    if (self.missionTextEditChaged):
      self.missionTextEditChaged = False
      self.ui.mission_edit.setPlainText(self.missionTextEdit)
    if (self.missionTextMsgChaged):
      self.missionTextMsgChaged = False
      self.ui.mission_edit_error.setPlainText(self.missionTextMsg)
    if (self.missionDataRead or self.mLinesNewData or self.missionTextMsgChaged):
      self.missionDataRead = False
      self.missionTextMsgChaged = False
      self.lock.acquire()
      #if (not self.missionManually):
        #self.ui.main_mission_2.setValue(self.mission)
      #self.ui.mission_name.setText(self.missionName)
      if (self.missionThreadState > 0 or self.missionLineState > 0):
        self.ui.main_mission_state.setText("m " + str(self.missionThreadState) + ":" + str(self.missionLineState))
      else:
        self.ui.main_mission_state.setText("m " + str(self.missionState) + ":"  + str(self.missionLineState))
      #if (self.mLinesNewData):
        #self.mLinesNewData = False
        #j = 1
        #for i in self.mLines:
          #if (not i.showed):
            #i.showed = True
            #if i.threadUse:
              #self.ui.mission_edit.append(i.toString())
            #else:
              #self.ui.mission_edit.append("    " + i.toString())
            #j = j + 1
      #self.ui.main_status_while_running.setChecked(self.statusWhileRunning)
      self.lock.release()
    pass
  def dataChangedManually(self):
    self.missionManually = True
  def saveMissionToRegbotMis(self):
    self.saveMissionToFile("regbot.mis")
  def loadMissionFromRegbotMis(self):
    self.loadMissionFromFile("regbot.mis")
  def saveMissionToFile(self, filename):
    if True: #try:
      f = open(filename, "w");
      f.write(self.ui.mission_edit.toPlainText())
      f.close()
      self.ui.statusbar.showMessage("Save mission file " + str(filename), 3000)
    #except:
      #self.ui.statusbar.showMessage("Failed to save mission file " + str(filename) + " !", 3000)
  def loadMissionFromFile(self, filename):
    if True: #try:
      f = open(filename, "r");
      #self.ui.mission_edit.clear()
      self.missionTextEdit = f.read()
      self.missionTextEditChaged = True
      #self.ui.mission_edit.setPlainText(f.read())
      f.close()
      self.ui.statusbar.showMessage("Loaded mission file from " + str(filename), 3000)
    #except:
      #self.ui.statusbar.showMessage("Failed to open mission file " + str(filename) + " !", 3000)
    pass
  def loadMissionFrom(self):
    #print("#LoadMissionFrom 1 last '" + self.lastFileName[0] + "'")
    mis = QtGui.QFileDialog.getOpenFileName(self.robot.parent,'directory and filename for mission file', self.lastFileName[0], self.lastFileName[1])
    if (mis is not None and len(mis) > 0):
      self.loadMissionFromFile(mis[0])
      self.lastFileName = mis
    #print("#LoadMissionFrom 2 last '" + str(self.lastFileName[0]) + "'")
  def saveMissionAs(self):
    #print("#LoadMissionFrom 3 last '" + str(self.lastFileName[0]) + "'")
    mis = QtGui.QFileDialog.getSaveFileName(self.robot.parent,'directory and filename for mission file', self.lastFileName[0], self.lastFileName[1])
    if (mis is not None and len(mis) > 0):
      self.saveMissionToFile(mis[0])
      self.lastFileName = mis
    #print("# mis[0] = '" + str(mis[0]) + "'")
    #print("# mis[1] = '" + str(mis[1]) + "'")
    #print("#LoadMissionFrom 4 last '" + str(self.lastFileName[0]) + "'")
  def checkMission(self):
    error = False
    self.mLines = []
    m = self.ui.mission_edit.toPlainText().strip()
    lines = m.split('\n')
    for i in range(0,len(lines)):
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
      self.missionTextMsg += str(self.checkNr) + ": Error line " + str(i+1) + " " + es + "\n"
    else:
      self.missionTextMsg += str(self.checkNr) + ": No Error found (in " + str(len(lines)) + " lines)" + "\n"
    self.missionTextMsgChaged = True
    self.checkNr = self.checkNr + 1
  #
  ## send compiled user mission to robot - called by top "save" button on mission pane
  def sendToRobot(self):
    self.checkMission()
    if (len(self.mLines) > 0):
      print("mission is OK, sending!")
      # clear old mission
      self.robot.conWrite("<clear\n")
      for ml in range(0, len(self.mLines)):
        self.robot.conWrite("<add " + self.mLines[ml].toString() + "\n")
    #self.conWrite("u4\r\n") # static robot info
    #self.conWrite("S=" + str(self.ui.main_push_interval.value()) + "\r\n")
    #self.ui.main_mission_2.setValue(0)
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
          # self.ui.mission_edit_error.setPlainText("Error line " + str(len(self.mLines)) + " " + es)
        else:
          self.mLines.append(ml)
          if ml.threadUse:
            self.missionTextMsg += ml.toString() + '\n'
          else:
            self.missionTextMsg += "    " + ml.toString() + '\n'
          self.missionTextMsgChaged = True
          #ml.showed = False
          #self.mLinesNewData = True
          #print("This line should now be appended to mission_window: " + line)
      used = True
    return used
  #
  def clearRxField(self):
    self.missionTextMsg = ""
    self.missionTextMsgChaged = True
  #
  def getFromRobot(self):
    self.missionTextMsg += "# Got from robot:\n"
    self.missionTextMsgChaged = True
    #self.ui.mission_edit.append("# got from robot:")
    if self.robot.info.talkToBridge:
      self.robot.conWrite("<m subscribe 6\n")
      self.robot.conWrite("robot <get\n")
    else:
      self.robot.conWrite("<get\n")
  def helpbox(self):
    if (self.about_box == None):
      self.about_box = QtGui.QMessageBox(self.robot.parent)
      self.about_box.setSizeGripEnabled(True)
      self.about_box.setText('''<p><span style=" font-size:20pt;">
                Mission setup</span></p>
                <p>See also the <a href="http://rsewiki.elektro.dtu.dk/index.php/Mission">REGBOT wiki page</a> for more details</p>
                <p>Mission specification consist of mission lines,
                   each line consist of two (lower case) parts divided by a ':'</p>
                <p><b>drive values : continue conditions</b>  (conditions are OR'ed)</p>
                <p>e.g.:<br/> vel=-0.2 : dist=1.2, time=6<br />
                Drive backwards at 0.2m/s for 1.2 meter or max 6 seconds.</p>
                
                <p>
                <b>Drive values</b><br />
                VEL is velocity in m/s - positive is forward.<br/>
                ACC is acceleration limit in m/s2.<br/>
                TOPOS is target position for this missionline + is forward (vel is max vel).<br/>
                TR is turnradius in metre - always positive.<br/>
                EDGER, EDGEL follow Right/Left edge of line at -2..2 (in cm).<br/>
                WHITE set to 1 if follow-line tape is white, else black.<br/>
                LOG is log interval in milliseconds.<br/>
                BAL is balancing, uses last value if omitted.<br/>
                IRSENSOR is IR-sensor to use for control (1=Wall, 2=Velocity).<br/>
                IRDist is IR-distance to hold (selsor 1 to wall, sensor 2 to leader).<br/>
                HEAD sets the reference heading in pose coordinates.<br/>
                SERVO servo (1..5), PSERVO position (-1000..1000), VSERVO servo speed 1=slow 10=fast.<br/>
                LABEL label number that can be used by GOTO.<br />
                GOTO is a jump to the label number given. Can be limited by COUNT or any other condition.<br/>
                EVENT generate an event 0..31, event=0 means stop mission.<br/>
                CHIRP may be used to make a frequency scan with this amplitude, LOG must be > 3 (samples per cycle).<br/>
                <b>Continue conditions</b><br />
                DIST driven distance in this mission line - positive meters.<br/>
                VEL  robot velocity (positive is forward).<br/>
                TURN angle turned (max) - degrees, positive is CCV.<br />
                HEAD absolute angle in degrees, can be used with '=' within +/-3 deg, &lt;, &gt; (beware of 180 deg folding).<br/>
                TIME max time in this mission line - positive seconds<br />
                COUNT used with GOTO, GOTO is skipped when count is reached.<br />
                XL crossing line, value is 0..20, use '=', '&lt;' or '&gt;'<br />
                LV line valid confidence 0..20 (0=no line, increase on detect up to 20), use '=', '&lt;' or '&gt;'<br/>
                IR1, IR2 distance from side IR sensor.<br/>
                TILT tilt angle (0 is balance point).<br/>
                LAST condition that ended last line with a condition.<br/>
                EVENT test for event.<br/>
                If no condition, then continues right away.<br/>
                </p>''');
      self.about_box.setWindowTitle("Regbot mission keywords")
      #self.about_box.setMinimumHeight(0)
      #self.about_box.setMaximumHeight(16777215)
      #self.about_box.setMinimumWidth(0)
      #self.about_box.setMaximumWidth(16777215)
      self.about_box.setWindowModality(QtCore.Qt.NonModal)
      #self.about_box.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)

    self.about_box.show()

  def justConnected(self):
    self.lastTab = "none"
