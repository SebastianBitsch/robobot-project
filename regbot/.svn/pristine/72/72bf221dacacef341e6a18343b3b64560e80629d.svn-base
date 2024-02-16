import threading
import numpy as np
import time

class ServoControl(object):
    
    ServoUsage = [False, False, False, False, False]
    ServoPosition = [0,0,0,0,0]
    ServoSpeedSetting = [0,0,0,0,0]
    Servo1Steering = False
    SteerOffset = 0
    SteerWheelDist = 0.135
    SteerScale = 90
    b_readData = False
    b_readData1 = False
    lastIndex = -1;
    
    def __init__(self, robot):
        self.robot = robot
        self.ui = robot.ui
        
    def timerUpdate(self):
        # subscribe to data
        if self.ui.tabWidget.currentIndex() == 4 and self.lastIndex != 4:
            self.robot.connectionWrite("svo subscribe 6\n")
            self.robot.connectionWrite("sv1 subscribe 6\n")
            self.robot.connectionWrite("svo\n")
        elif self.lastIndex == 4:
          # moved away from this tab
          self.robot.connectionWrite("svo subscribe 0\n")
          self.robot.connectionWrite("sv1 subscribe 0\n")
        self.lastIndex = self.ui.tabWidget.currentIndex()
        if self.b_readData:
            self.ui.enableServo1_checkBox.setChecked(self.ServoUsage[0])
            self.ui.enableServo2_checkBox.setChecked(self.ServoUsage[1])
            self.ui.enableServo3_checkBox.setChecked(self.ServoUsage[2])
            self.ui.enableServo4_checkBox.setChecked(self.ServoUsage[3])
            self.ui.enableServo5_checkBox.setChecked(self.ServoUsage[4])
            
            self.ui.servo1_current_pos.setText(str(self.ServoPosition[0]))
            self.ui.servo2_current_pos.setText(str(self.ServoPosition[1]))
            self.ui.servo3_current_pos.setText(str(self.ServoPosition[2]))
            self.ui.servo4_current_pos.setText(str(self.ServoPosition[3]))
            self.ui.servo5_current_pos.setText(str(self.ServoPosition[4]))  
            self.b_readData = False
               
        if self.ui.enableServo1_checkBox.isChecked():
            self.ui.Servo1_Pos_dial.valueChanged.connect(self.DoServoPosition)
        if self.ui.enableServo2_checkBox.isChecked():
            self.ui.Servo2_Pos_dial.valueChanged.connect(self.DoServoPosition)
        if self.ui.enableServo2_checkBox.isChecked():
            self.ui.Servo2_Pos_dial.valueChanged.connect(self.DoServoPosition)
        if self.ui.enableServo3_checkBox.isChecked():
            self.ui.Servo3_Pos_dial.valueChanged.connect(self.DoServoPosition)
        if self.ui.enableServo4_checkBox.isChecked():
            self.ui.Servo4_Pos_dial.valueChanged.connect(self.DoServoPosition)    
        if self.ui.enableServo5_checkBox.isChecked():
            self.ui.Servo5_Pos_dial.valueChanged.connect(self.DoServoPosition)    
        
        if (self.b_readData1):
            self.ui.Servo1Steering_checkBox.setChecked(self.Servo1Steering)
            self.ui.val_servo1_offset.setValue(self.SteerOffset)
            self.ui.val_steer_distance.setValue(self.SteerWheelDist)
            self.ui.val_servo1_scale.setValue(self.SteerScale)   
            self.b_readData1 = False
          
          
        
    def readData(self, MESSAGE):
        used = True
        try:
            if MESSAGE[0] == "sv0":
                self.b_readData = True
                self.ServoUsage[0] = int(MESSAGE[1],0)
                self.ServoPosition[0] = int(MESSAGE[2],0)
                self.ServoSpeedSetting[0] = int(MESSAGE[3],0)
                self.ServoUsage[1] = int(MESSAGE[4],0)
                self.ServoPosition[1] = int(MESSAGE[5],0)
                self.ServoSpeedSetting[0] = int(MESSAGE[6],0)
                self.ServoUsage[2] = int(MESSAGE[7],0)
                self.ServoPosition[2] = int(MESSAGE[8],0)
                self.ServoSpeedSetting[2] = int(MESSAGE[9],0)
                self.ServoUsage[3] = int(MESSAGE[10],0)
                self.ServoPosition[3] = int(MESSAGE[11],0)
                self.ServoSpeedSetting[3] = int(MESSAGE[12],0)
                self.ServoUsage[4] = int(MESSAGE[13],0)
                self.ServoPosition[4] = int(MESSAGE[14],0)
                self.ServoSpeedSetting[4] = int(MESSAGE[15],0)
                print(MESSAGE)
            elif MESSAGE[0] == 'sv1':
                self.b_readData1 = True
                self.Servo1Steering = int(MESSAGE[1],0)
                self.SteerOffset = int(MESSAGE[2],0)
                self.SteerWheelDist = float(MESSAGE[3])
                self.SteerScale = float(MESSAGE[4])  
                print(MESSAGE)
            # else:
            #     used = False
                
                
            
        except:
            print("UServo: data read error - skipped a " + MESSAGE[0] + " from " + MESSAGE)
            pass   
        return used
  
    
    def DoServoPosition(self):
        MESSAGE = "svo {} {} 0 {} {} 0 {} {} 0 {} {} 0 {} {} 0\n".format(
                                                                int(self.ui.enableServo1_checkBox.isChecked()),
                                                                self.ui.Servo1_Pos_dial.value(),
                                                                int(self.ui.enableServo2_checkBox.isChecked()),
                                                                self.ui.Servo2_Pos_dial.value(),
                                                                int(self.ui.enableServo3_checkBox.isChecked()),
                                                                self.ui.Servo3_Pos_dial.value(),
                                                                int(self.ui.enableServo4_checkBox.isChecked()),
                                                                self.ui.Servo4_Pos_dial.value(),
                                                                int(self.ui.enableServo5_checkBox.isChecked()),
                                                                self.ui.Servo5_Pos_dial.value()
                                                                )
        self.robot.connectionWrite(MESSAGE)
        pass
    
    def applyServoPosition(self):
        MESSAGE = "svo {} {} 0 {} {} 0 {} {} 0 {} {} 0 {} {} 0\n".format(
                                                                int(self.ui.enableServo1_checkBox.isChecked()),
                                                                self.ui.manualServoPos1_Val.text(),
                                                                int(self.ui.enableServo2_checkBox.isChecked()),
                                                                self.ui.manualServoPos2_Val.text(),
                                                                int(self.ui.enableServo3_checkBox.isChecked()),
                                                                self.ui.manualServoPos3_Val.text(),
                                                                int(self.ui.enableServo4_checkBox.isChecked()),
                                                                self.ui.manualServoPos4_Val.text(),
                                                                int(self.ui.enableServo5_checkBox.isChecked()),
                                                                self.ui.manualServoPos5_Val.text()
                                                                )
        self.robot.connectionWrite(MESSAGE)
        pass
    
    def applyServoSteering(self):
          s = "sv1 {} {} {} {}\n".format(
                                    int(self.ui.Servo1Steering_checkBox.isChecked()),
                                    int(self.ui.val_servo1_offset.value()),
                                    self.ui.val_steer_distance.value(),
                                    self.ui.val_servo1_scale.value()
                                    )
          self.robot.conWrite(s)
          pass
