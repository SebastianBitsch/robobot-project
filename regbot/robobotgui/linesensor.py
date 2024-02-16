import threading
import numpy as np
import time


class LineSensor(object):
    lineValue = [0,0,0,0,0,0,0,0]
    lineMaxWhite = [0,0,0,0,0,0,0,0]
    lineMaxBlack = [0,0,0,0,0,0,0,0]
    lineWhite = False
    LineSensor_enabled = True
    edgeLeft = 0.0
    edgeRight = 0.0
    edgeLeftValid = False
    edgeRightValid = False
    followLeft = True
    wideSensor = False;
    crossingDetectLimit = 4
    crossingWhiteLine = 0.0
    swapLeftRight = False;
    dataReadLiv = False
    dataReadLip = False
    dataReadBlackWhite = False
    inUpdate = False
    
    def __init__(self, robot, parent):
        self.robot = robot
        self.ui = robot.ui
        self.parent = parent
        
    def readData(self, MESSAGE):
        used = True
        try:
            if MESSAGE[0] == "liv":
                self.dataReadLiv = True
                if self.ui.ls_show_normalized.isChecked():
                    maxraw = self.ui.line_disp_max_value.value()
                    self.lineValue[0] = maxraw * (int(MESSAGE[1],0) - self.lineMaxBlack[0])/(self.lineMaxWhite[0] - self.lineMaxBlack[0])
                    self.lineValue[1] = maxraw * (int(MESSAGE[2],0) - self.lineMaxBlack[1])/(self.lineMaxWhite[1] - self.lineMaxBlack[1])
                    self.lineValue[2] = maxraw * (int(MESSAGE[3],0) - self.lineMaxBlack[2])/(self.lineMaxWhite[2] - self.lineMaxBlack[2])
                    self.lineValue[3] = maxraw * (int(MESSAGE[4],0) - self.lineMaxBlack[3])/(self.lineMaxWhite[3] - self.lineMaxBlack[3])
                    self.lineValue[4] = maxraw * (int(MESSAGE[5],0) - self.lineMaxBlack[4])/(self.lineMaxWhite[4] - self.lineMaxBlack[4])
                    self.lineValue[5] = maxraw * (int(MESSAGE[6],0) - self.lineMaxBlack[5])/(self.lineMaxWhite[5] - self.lineMaxBlack[5])
                    self.lineValue[6] = maxraw * (int(MESSAGE[7],0) - self.lineMaxBlack[6])/(self.lineMaxWhite[6] - self.lineMaxBlack[6])
                    self.lineValue[7] = maxraw * (int(MESSAGE[8],0) - self.lineMaxBlack[7])/(self.lineMaxWhite[7] - self.lineMaxBlack[7])
                else:    
                    self.lineValue[0] = int(MESSAGE[1],0)
                    self.lineValue[1] = int(MESSAGE[2],0)
                    self.lineValue[2] = int(MESSAGE[3],0)
                    self.lineValue[3] = int(MESSAGE[4],0)
                    self.lineValue[4] = int(MESSAGE[5],0)
                    self.lineValue[5] = int(MESSAGE[6],0)
                    self.lineValue[6] = int(MESSAGE[7],0)
                    self.lineValue[7] = int(MESSAGE[8],0)
                #print(MESSAGE)
            elif MESSAGE[0] == "liw":
                self.dataReadLiw = True
                self.dataReadBlackWhite = True
                self.lineMaxWhite[0] = int(MESSAGE[1],0)
                self.lineMaxWhite[1] = int(MESSAGE[2],0)
                self.lineMaxWhite[2] = int(MESSAGE[3],0)
                self.lineMaxWhite[3] = int(MESSAGE[4],0)
                self.lineMaxWhite[4] = int(MESSAGE[5],0)
                self.lineMaxWhite[5] = int(MESSAGE[6],0)
                self.lineMaxWhite[6] = int(MESSAGE[7],0)
                self.lineMaxWhite[7] = int(MESSAGE[8],0)
               # print(MESSAGE)
            elif MESSAGE[0] == "lib":
                self.dataReadLib = True
                self.dataReadBlackWhite = True
                self.lineMaxBlack[0] = int(MESSAGE[1],0)
                self.lineMaxBlack[1] = int(MESSAGE[2],0)
                self.lineMaxBlack[2] = int(MESSAGE[3],0)
                self.lineMaxBlack[3] = int(MESSAGE[4],0)
                self.lineMaxBlack[4] = int(MESSAGE[5],0)
                self.lineMaxBlack[5] = int(MESSAGE[6],0)
                self.lineMaxBlack[6] = int(MESSAGE[7],0)
                self.lineMaxBlack[7] = int(MESSAGE[8],0)
                #print(MESSAGE)
            elif MESSAGE[0] == "lip":
                self.LineSensor_enabled = int(MESSAGE[1],0)
                self.lineWhite = int(MESSAGE[2],0)
                self.edgeLeft = float(MESSAGE[3])
                self.edgeLeftValid = int(MESSAGE[4],0)
                self.edgeRight = float(MESSAGE[5])
                self.edgeRightValid = int(MESSAGE[6],0)
                self.crossingWhiteLine = int(MESSAGE[10],0)
                self.crossingDetectLimit = float(MESSAGE[14])
                #print(MESSAGE)
            else:
                used = False    
        except:
            print("data read error - skipped packege " + MESSAGE[0] + " length=" + str(len(MESSAGE)))
            pass
        return used

    def lineSensorConfigUpdate(self):
        self.robot.connectionWrite("lip=%d %d %d %d %g %d %d\n" % (  
                                                            self.ui.enableLineSensor_checkBox.isChecked(),
                                                            self.ui.ls_line_white.isChecked(),
                                                            self.ui.ls_power_high.isChecked(),
                                                            False,
                                                            self.ui.ls_crossing_detect.value(),
                                                            True,
                                                            self.ui.ls_swap_left_right.isChecked()
            ))
        # print("lip=",   self.ui.enableLineSensor_checkBox.isChecked(),
        #                 self.ui.ls_line_white.isChecked(),
        #                 self.ui.ls_power_high.isChecked(),
        #                 False,
        #                 self.ui.ls_crossing_detect.value(),
        #                 True,
        #                 self.ui.ls_swap_left_right.isChecked())

    def EnableLineSensor(self):
        self.lineSensorConfigUpdate()
        
    def EnableWhiteLine(self):        
        self.lineSensorConfigUpdate()  
    
    def EnableHighPower(self):        
        self.lineSensorConfigUpdate()    
    
    def EnableSwapLSSides(self):        
        self.lineSensorConfigUpdate()     
        
    def MaxRawChanged(self):
        val = self.ui.line_disp_max_value.value()
        self.ui.line_bar_1.setMaximum(val)
        self.ui.line_bar_2.setMaximum(val)
        self.ui.line_bar_3.setMaximum(val)
        self.ui.line_bar_4.setMaximum(val)
        self.ui.line_bar_5.setMaximum(val)
        self.ui.line_bar_6.setMaximum(val)
        self.ui.line_bar_7.setMaximum(val)
        self.ui.line_bar_8.setMaximum(val)
        
    def calibrateWhite(self):
        self.robot.connectionWrite("robot licw\n")

    def calibrateBlack(self):
        self.robot.connectionWrite("robot licb\n")
    

    def timerUpdate(self):
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
        self.MaxRawChanged()
        
        if self.ui.tabWidget.currentIndex() == 1:
            self.robot.connectionWrite("liw subscribe 6\n")
            self.robot.connectionWrite("lib subscribe 6\n") 
            self.robot.connectionWrite("liv subscribe 6\n")
            self.robot.connectionWrite("lip subscribe 2\n")
            self.robot.connectionWrite("u9\n")
            self.robot.connectionWrite("u10\n")
            if (self.ui.ls_show_normalized.isChecked()):
                self.robot.connectionWrite("u11\n")
            else:
                self.robot.connectionWrite("u12\n")
        else:
          self.robot.connectionWrite("liw subscribe 0\n")
          self.robot.connectionWrite("lib subscribe 0\n") 
          self.robot.connectionWrite("liv subscribe 0\n") 
          self.robot.connectionWrite("lip subscribe 0\n") 
          self.robot.connectionWrite("sub 0 2 3\n")
        
        maxraw = self.ui.line_disp_max_value.value()
        #print(maxraw)
        if (self.dataReadLiv):
            self.dataReadLiv = False
            self.inUpdate = True
            if (self.lineValue[0] > maxraw):
                self.ui.line_bar_1.setValue(maxraw)
            elif self.lineValue[0] < 0:
                self.ui.line_bar_1.setValue(0)
            else:
                self.ui.line_bar_1.setValue(self.lineValue[0])
            if (self.lineValue[1] > maxraw):
                self.ui.line_bar_2.setValue(maxraw)
            elif self.lineValue[1] < 0:
                self.ui.line_bar_2.setValue(0)
            else:
                self.ui.line_bar_2.setValue(self.lineValue[1])
            if (self.lineValue[2] > maxraw):
                self.ui.line_bar_3.setValue(maxraw)
            elif self.lineValue[2] < 0:
                self.ui.line_bar_3.setValue(0)
            else:
                self.ui.line_bar_3.setValue(self.lineValue[2])
            if (self.lineValue[3] > maxraw):
                self.ui.line_bar_4.setValue(maxraw)
            elif self.lineValue[3] < 0:
                self.ui.line_bar_4.setValue(0)
            else:
                self.ui.line_bar_4.setValue(self.lineValue[3])
            if (self.lineValue[4] > maxraw):
                self.ui.line_bar_5.setValue(maxraw)
            elif self.lineValue[4] < 0:
                self.ui.line_bar_5.setValue(0)
            else:
                self.ui.line_bar_5.setValue(self.lineValue[4])
            if (self.lineValue[5] > maxraw):
                self.ui.line_bar_6.setValue(maxraw)
            elif self.lineValue[5] < 0:
                self.ui.line_bar_6.setValue(0)
            else:
                self.ui.line_bar_6.setValue(self.lineValue[5])
            if (self.lineValue[6] > maxraw):
                self.ui.line_bar_7.setValue(maxraw)
            elif self.lineValue[6] < 0:
                self.ui.line_bar_7.setValue(0)
            else:
                self.ui.line_bar_7.setValue(self.lineValue[6])
            if (self.lineValue[7] > maxraw):
                self.ui.line_bar_8.setValue(maxraw)
            elif self.lineValue[7] < 0:
                self.ui.line_bar_8.setValue(0)
            else:
                self.ui.line_bar_8.setValue(self.lineValue[7])
                
            self.ui.ls_left_side.setValue(self.edgeLeft)
            self.ui.ls_right_side.setValue(self.edgeRight)
            
            if self.edgeLeftValid > 5 or self.edgeRightValid > 5 :
                self.ui.ls_line_valid_cnt.setText(str("TRUE"))
            else:    
                self.ui.ls_line_valid_cnt.setText(str("FALSE"))
            
            if self.crossingWhiteLine > 5 :
                self.ui.ls_crossing_cnt.setText(str("TRUE"))
            else:    
                self.ui.ls_crossing_cnt.setText(str("FALSE"))
            
            val = 100 - (self.edgeLeft + 2.0) * 20.0
            if (val < 0):
              val = 0
            elif val >100:
              val = 100
            if self.edgeLeftValid:
              self.ui.ls_left_edge.setValue(int(val))
            else:
              self.ui.ls_left_edge.setValue(-3)
              
            val = (self.edgeRight + 2.0) * 20.0
            if (val < 0):
              val = 0
            elif val >100:
              val = 100
            if self.edgeRightValid:
              self.ui.ls_right_edge.setValue(int(val))
            else:
              self.ui.ls_right_edge.setValue(-3)
              
        self.inUpdate = False
    
