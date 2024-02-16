import threading
import time
#import pyqtgraph as pg

## Robot info from previous version 

class URobotInfo(object):
  wheelbase = 0.0
  gear = 0.0
  pulsePerRev = 0
  wheelLRadius = 0.0
  wheelRRadius = 0.0
  version = ""
  robotID = 0
  robotHWtype = 2 # type 1 is old with no wifi and line sensor, 2 if with satellite PCB for wifi and power, 3 is with power mgmt on board
  balanceOffset = 0.0
  batteryUse = True
  reverseMotor = False
  reverseEncoderLeft = False
  reverseEncoderRight = False
  batteryIdleVolt = 9.9
  name = "empty"
  gotRobotName = False
  wifiIP = "0.0.0.0"
  wifiMAC= "00:00:00:00:00:00"
  #wifiport = 24001

class UInfo(object):
  dataRead = True
  dataWifi = True
  dataClient = True
  robotID = 0
  gyroOK = False
  lock = threading.RLock()
  robots = [URobotInfo]
  thisRobot = robots[0]
  inEdit = False
  wifiUse = False
  wifiSSID = "aaa"
  wifiPW = ""
  wifiGotIP = False
  wifiPortOpen = False
  wifiInEdit = False
  wifiSetupState = -1;
  wifiSleep = False;
  wifiPort = 24001
  clientRxCnt = [0,0,0,0,0]
  clientTxCnt = [0,0,0,0,0]
  wifiGood = 0
  #wifiLost = 0
  wifiLoss = 0.0
  lastDataRequestTime = time.time()
  lastDataRequest = 0
  lastAliveTime = 0.0
  talkToBridge = False
  talkToBridgeOld = False;
  lastTab = ""
  #  
  def __init__(self, robot):
    self.robot = robot
    self.ui = robot.ui

  def gotAliveMsg(self):
    self.lastAliveTime = time.time()
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
        if gg[0] == "rid":
            self.robotID = int(gg[1],10)
            self.thisRobot = self.getRobot(self.robotID)
            self.thisRobot.wheelbase = float(gg[2])
            self.thisRobot.gear = float(gg[3])
            self.thisRobot.pulsePerRev = int(gg[4], 10)
            self.thisRobot.wheelLRadius = float(gg[5])
            self.thisRobot.wheelRRadius = float(gg[6])
            self.thisRobot.balanceOffset = float(gg[7])
            flags = int(gg[8])
            self.thisRobot.batteryUse = flags & 1
            self.thisRobot.reverseMotor = flags & 2
            self.thisRobot.reverseEncoderLeft = flags & 4
            self.thisRobot.reverseEncoderRight = flags & 8
            self.thisRobot.batteryIdleVolt = float(gg[9])
            try:
                self.thisRobot.robotHWtype = float(gg[10])
                self.thisRobot.name = gg[11]
            except:
                self.thisRobot.robotHWtype = 2;
                self.thisRobot.name = gg[10]
            self.thisRobot.gotRobotName = True
            self.dataRead = True
            #print(gg)
        elif gg[0] == "bridge":
            print("Talk to bridge TRUE")
            self.talkToBridge = True
        self.lock.release()

  #
  def justConnected(self):
    self.robot.connectionWrite("u4\n")
    self.robot.connectionWrite("u0\n")
    
  #
  def timerUpdate(self):
    if (self.talkToBridge != self.talkToBridgeOld):
        self.talkToBridgeOld = self.talkToBridge
        if (self.talkToBridge):
            self.robot.connectionWrite("# subscribe 6\n")
            self.robot.connectionWrite("hbt subscribe 3\n")
     

    # if (self.dataRead):
    #     self.dataRead = False
    #     self.lock.acquire()
    #     if (not self.inEdit):
    #         # set new values
    #         self.ui.Robot_GearRatio.setProperty("value", self.thisRobot.gear)
    #         self.ui.Robot_PulsePerRev.setValue(self.thisRobot.pulsePerRev)
    #         self.ui.Robot_WheelRadius_Left.setValue(self.thisRobot.wheelLRadius)
    #         self.ui.Robot_WheelRadius_Right.setValue(self.thisRobot.wheelRRadius)
    #         self.ui.RobotHW_type.setValue(self.thisRobot.robotHWtype)
    #         self.ui.Robot_BalanceOffset.setValue(self.thisRobot.balanceOffset)
    #         #self.ui.save_id_on_robot.setEnabled(False)
    #         self.ui.Robot_WheelBase.setValue(self.thisRobot.wheelbase)
    #         self.ui.RobotID.setValue(self.robotID)
    #         #self.ui.robot_id_main.setText(self.thisRobot.name + " (" + str(self.robotID) + ")")
    #         self.ui.Robot_OnBattery.setChecked(self.thisRobot.batteryUse)
    #         self.ui.Robot_RevMotor.setChecked(self.thisRobot.reverseMotor)
    #         self.ui.Robot_RevEncoder_Left.setChecked(self.thisRobot.reverseEncoderLeft)
    #         self.ui.Robot_RevEncoder_Right.setChecked(self.thisRobot.reverseEncoderRight)
    #         self.ui.Robot_BatteryIdleVolts.setValue(self.thisRobot.batteryIdleVolt)
    #         # and buttons
    #         self.ui.save_id_on_robot.setEnabled(False)
    #         self.ui.robot_cancel.setEnabled(False)
    #         self.ui.Robot_edit.setEnabled(True)
    #     else:
    #         self.ui.save_id_on_robot.setEnabled(True)
    #         self.ui.robot_cancel.setEnabled(True)
    #         self.ui.Robot_edit.setEnabled(False)
    #     self.lock.release()


  # 
  def dataChangedManually(self):
    # robot static parameters
    if (not self.robot.timerUpdate):
      self.lock.acquire()
      self.inEdit = True
      self.lock.release()

  #
  def cancelEdit(self):
    self.inEdit = False;
