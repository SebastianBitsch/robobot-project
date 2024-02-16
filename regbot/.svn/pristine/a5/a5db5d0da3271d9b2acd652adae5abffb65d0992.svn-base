import threading
import numpy as np
import time

class IR_Distance(object):
    
    b_readData = False
    SensorsOn = False
    SensorInstalled = False
    inEdit = False
    Sensor1_dist = 0
    Sensor2_dist = 0
    IR_DataRaw = [0, 0]
    IR_Cal13cm = [3000, 3000]
    IR_Cal50cm = [480, 480]
    lock = threading.RLock()
    
    def __init__(self, robot):
        self.robot = robot
        self.ui = robot.ui
        
    def timerUpdate(self):    
        
        # self.ui.checkBox_ir_use.setChecked(self.SensorsOn)
        # self.ui.checkBox_ir_installed.setChecked(self.SensorInstalled)
        
        if self.ui.tabWidget.currentIndex() == 2:
            self.robot.connectionWrite("irc subscribe 2\n")
        else:
            self.robot.connectionWrite("irc subscribe 0\n")
          
        if (self.b_readData):
            self.b_readData = False
            self.ui.ir_d1_meters.setText(str(self.Sensor1_dist))
            self.ui.ir_d2_meters.setText(str(self.Sensor2_dist))
            self.ui.ir_bar_1.setValue(self.IR_DataRaw[0])
            self.ui.ir_bar_2.setValue(self.IR_DataRaw[1])
            self.ui.ir_d1_raw.setValue(self.IR_DataRaw[0])
            self.ui.ir_d2_raw.setValue(self.IR_DataRaw[1])
            # if not self.inEdit:
            #     self.ui.ir_d1_20cm.setValue(self.IR_Cal13cm[0])
            #     self.ui.ir_d2_20cm.setValue(self.IR_Cal13cm[1])
            #     self.ui.ir_d1_80cm.setValue(self.IR_Cal50cm[0])
            #     self.ui.ir_d2_80cm.setValue(self.IR_Cal50cm[1])
            
        self.ui.ir_apply.clicked.connect(self.ApplyCalibrationParameters)  
        #self.ui.ir_edit.clicked.connect(self.inEditMode)
        
        
    def inEditMode(self):
        self.inEdit = True
                
    def readData(self, MESSAGE):    
        used = True
        self.lock.acquire()
        try:
            if MESSAGE[0] == "irc":
                self.b_readData = True
                self.Sensor1_dist = float(MESSAGE[1])
                self.Sensor2_dist = float(MESSAGE[2])
                self.IR_DataRaw[0] = int(MESSAGE[3],0)
                self.IR_DataRaw[1] = int(MESSAGE[4],0)
                self.IR_Cal13cm[0] = int(MESSAGE[5],0)
                self.IR_Cal50cm[0] = int(MESSAGE[6],0)
                self.IR_Cal13cm[1] = int(MESSAGE[7],0)
                self.IR_Cal50cm[1] = int(MESSAGE[8],0)
                self.SensorsOn = int(MESSAGE[9],0)
                self.SensorInstalled = int(MESSAGE[10],0)
                print(MESSAGE)
            else:
                used = False
        except:
            pass   
        return used
    
    def ApplyCalibrationParameters(self):
        self.robot.connectionWrite("irc=%g %g %g %g %d %d\n" % (
                                        self.ui.ir_d1_20cm.value(), 
                                        self.ui.ir_d1_80cm.value(), 
                                        self.ui.ir_d2_20cm.value(), 
                                        self.ui.ir_d2_80cm.value(),
                                        int(self.ui.checkBox_ir_use.isChecked()),
                                        int(self.ui.checkBox_ir_installed.isChecked())
                                        ))
        self.inEdit = False