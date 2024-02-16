## Module definitions and library importing
import sys
import os
import time
import timeit
import threading
import math
import socket
import keyboard
import cv2
import numpy as np 
from PyQt5 import QtCore, QtGui, QtWidgets # Importing Python Qt widgets and Gui
from PyQt5.QtGui import QColor, QPalette, QImage, QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
from RegbotGUI_V1_9 import Ui_MainWindow
from linesensor import LineSensor
from ServoControl import ServoControl
from IR_DistanceSensors import IR_Distance
from RegbotStreamer import VideoThread, VideoRecordingThread
from Rinfo import UInfo


class GUIMainWindow(QtWidgets.QMainWindow):

    socket = socket.socket()
    wifiConnected = False
    guiRC_enabled = False
    keyboardRC_enabled = False
    camera_enabled = False
    ConsoleMessage = ""
    ConsoleMessagePush = False
    timerUpdate = False
    take_snapshot = False
    lastDataRequestTime = time.time()
    lastToogleTime = time.time()
    snapshot_image = 0
    toogleSignalON = False
    video_recording = False
    currentView = "none"
 
    def __init__(self):
        super(GUIMainWindow,self).__init__()
        print("Initializing Application...")
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.lineSensor = LineSensor(self, self.parent)
        self.servo = ServoControl(self)
        self.irdist = IR_Distance(self)
        self.VideoThread = VideoThread()
        self.VideoRecordingThread = VideoRecordingThread()
        
        self.setConnectFrameColor(self.ui.IP_Connect_frame,QtGui.QColor(200,0,0,255))
        self.ui.send_Cmd.clicked.connect(self.sendCmd)
        self.ui.connect_Cmd.clicked.connect(self.connectClient)
        self.ui.Disconnect_Cmd.clicked.connect(self.disconnectClient)
        self.ui.ConsoleClear_Cmd.clicked.connect(self.ConsoleClear)
        self.ui.ConsoleHelp_Cmd.clicked.connect(self.ConsoleHelp)
        
        self.ui.enableKeyboardRC_checkBox.clicked.connect(self.keyboardRC_enable)
        self.ui.enableGuiRC_checkBox.clicked.connect(self.guiRC_enable)
        self.ui.EnableCamera_checkBox.clicked.connect(self.camera_enable)
        self.ui.Camera_Snapshot_Cmd.clicked.connect(self.camera_snapshot)
        self.ui.Camera_SnapSave_Cmd.clicked.connect(self.snapshot_save)
        self.ui.Camera_ClearSnap_Cmd.clicked.connect(self.ClearSnap)
        self.ui.Camera_Record_Cmd.clicked.connect(self.video_StartRecording)
        self.ui.Camera_StopRecord_Cmd.clicked.connect(self.video_StopRecording)
        self.ui.connect_Cmd.clicked.connect(self.camera_enable)
        self.ui.Disconnect_Cmd.clicked.connect(self.camera_enable)
        self.ui.guiRC_Forward_Cmd.pressed.connect(self.guiRC_fwd)
        self.ui.guiRC_Forward_Cmd.released.connect(self.guiRC_idle)
        self.ui.guiRC_Reverse_Cmd.pressed.connect(self.guiRC_rev)
        self.ui.guiRC_Reverse_Cmd.released.connect(self.guiRC_idle)
        self.ui.guiRC_Right_Cmd.pressed.connect(self.guiRC_right)
        self.ui.guiRC_Right_Cmd.released.connect(self.guiRC_idle)
        self.ui.guiRC_Left_Cmd.pressed.connect(self.guiRC_left)
        self.ui.guiRC_Left_Cmd.released.connect(self.guiRC_idle)
        
        #Line Sensor Controls
        self.ui.enableLineSensor_checkBox.clicked.connect(self.lineSensor.EnableLineSensor)
        self.ui.ls_line_white.clicked.connect(self.lineSensor.EnableWhiteLine)
        self.ui.ls_power_high.clicked.connect(self.lineSensor.EnableHighPower)
        self.ui.ls_swap_left_right.clicked.connect(self.lineSensor.EnableSwapLSSides)
        
        self.ui.CalibrateWhite_Cmd.clicked.connect(self.lineSensor.calibrateWhite)
        self.ui.CalibrateBlack_Cmd.clicked.connect(self.lineSensor.calibrateBlack)
        
        # Servos
        self.ui.manualServoPos1_Cmd.clicked.connect(self.servo.applyServoPosition)
        self.ui.manualServoPos2_Cmd.clicked.connect(self.servo.applyServoPosition)
        self.ui.manualServoPos3_Cmd.clicked.connect(self.servo.applyServoPosition)
        self.ui.manualServoPos4_Cmd.clicked.connect(self.servo.applyServoPosition)
        self.ui.manualServoPos5_Cmd.clicked.connect(self.servo.applyServoPosition)
        
        # IR Distance
        self.ui.manualServoPos5_Cmd.clicked.connect(self.servo.applyServoPosition)
        
        
        self.timer = QtCore.QTimer(self)
        self.timer.start(50) # value in ms)
        self.timer.timeout.connect(self.keyboardRC)
        self.timer.timeout.connect(self.timerUpdate)                
        self.init()
        
    def init(self):
        self.info = UInfo(self)
        self.stop = threading.Event()
        self.ReadWiFi = threading.Thread(target=self.readReply, name="regbot_wifi_reader")
        self.ReadWiFi.start()
        self.ui.VideoRecording_Label.setHidden(True)
        self.update_image(0)  
        self.VideoThread = VideoThread()   
        self.VideoThread.change_pixmap_signal.connect(self.update_image)   
        self.VideoThread.stop()
        
    def camera_enable(self):  
        if self.ui.EnableCamera_checkBox.isChecked():
            self.camera_enabled = True
        else:
            self.camera_enabled = False
            self.video_StopRecording()

        if self.wifiConnected :  
            if self.camera_enabled and self.wifiConnected and self.ui.EnableCamera_checkBox.isChecked():
                print("Camera Enabled")
                self.VideoThread = VideoThread()
                self.VideoThread.change_pixmap_signal.connect(self.update_image)
                self.VideoThread.start()
                self.ConsoleMessage += "*Camera enabled\n"
                self.ConsoleMessagePush = True
            elif not self.camera_enabled and not self.ui.EnableCamera_checkBox.isChecked():    
                print("Camera Disabled")
                self.VideoThread.stop()
                self.ConsoleMessage += "*Camera disabled\n"
                self.ConsoleMessagePush = True 
        else:    
             self.VideoThread.stop()
             
    def camera_snapshot(self):
        if self.camera_enabled:
            self.take_snapshot = True
            
    def snapshot_save(self):    
        t = time.gmtime()
        if self.ui.Camera_SnapFilename.text() == "":
            filename = 'snapshot_' + str(t.tm_year) + str(t.tm_mon) + str(t.tm_mday) + "_" + str(t.tm_hour) + str(t.tm_min) + str(t.tm_sec) +'.jpg'
        else:
            filename = str(self.ui.Camera_SnapFilename.text() + '.jpg')
        cv2.imwrite(filename, self.snapshot_image) 
        
    def video_StartRecording(self):
        self.video_recording = True
        if self.video_recording and self.wifiConnected:
            self.VideoRecordingThread = VideoRecordingThread()
            self.VideoRecordingThread.change_pixmap_signal.connect(self.update_image)
            self.VideoRecordingThread.start()
            self.ConsoleMessage += "*Recording...\n"
            self.ConsoleMessagePush = True
    
    def toogleRecLabel(self):
        self.toogleSignalON = not self.toogleSignalON
        if self.toogleSignalON:
            self.ui.VideoRecording_Label.setHidden(True)
        else:
            self.ui.VideoRecording_Label.setHidden(False)
            
    def video_StopRecording(self):
        if self.video_recording and self.wifiConnected:
            self.VideoRecordingThread.stop()
            self.ConsoleMessage += "*Recording Stopping...\n"
            self.ConsoleMessagePush = True            
        self.ui.VideoRecording_Label.setHidden(True)
        self.video_recording = False     
        
    def update_image(self, cv_img):
        if not self.camera_enabled:
            cv_img = cv2.imread('nosignal.png')  
        qt_img = self.convert_cv_qt(cv_img)
        self.ui.Stream_Label.setPixmap(qt_img) 
        if self.camera_enabled and self.take_snapshot: 
            qt_img = self.convert_cv_qt(cv_img)
            self.snapshot_image = cv_img
            self.ui.Snapshot_Label.setPixmap(qt_img) 
            self.take_snapshot = False
    
    def ClearSnap(self):
        if self.ui.Camera_ClearSnap_Cmd:
            cv_img = cv2.imread('nosignal.png')
            qt_img = self.convert_cv_qt(cv_img)
            self.ui.Snapshot_Label.setPixmap(qt_img) 
            
        
    def convert_cv_qt(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(500, 500, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)    
        
    def terminate(self):
        self.stop.set()
        self.socket.close()
        
        
    def robotSettings(self):
        flags = self.ui.Robot_OnBattery.isChecked() + self.ui.Robot_RevMotor.isChecked() * 2 + self.ui.Robot_RevEncoder_Left.isChecked() * 4 + self.ui.Robot_RevEncoder_Right.isChecked() * 8
        self.connectionWrite("rid=%d %g %g %d %g %g %g %d %g %d\n" % (
          self.ui.RobotID.value(),
          self.ui.Robot_WheelBase.value(),
          self.ui.Robot_GearRatio.value(),
          self.ui.Robot_PulsePerRev.value(),
          self.ui.Robot_WheelRadius_Left.value(),
          self.ui.Robot_WheelRadius_Right.value(),
          self.ui.Robot_BalanceOffset.value(),
          flags,
          self.ui.Robot_BatteryIdleVolts.value(),
          self.ui.RobotHW_type.value()
          ))
        self.info.inEdit = False
  
        
    def setConnectFrameColor(self,frame,color):
        p = frame.palette()
        p.setColor(frame.backgroundRole(),color)
        frame.setPalette(p)

        
    def timerUpdate(self):
        if (self.ConsoleMessagePush):
            self.ConsoleMessagePush = False
            self.ui.CommandConsole_Window.setPlainText(str(self.ConsoleMessage))
            self.ui.CommandConsole_Window.verticalScrollBar().setValue(self.ui.CommandConsole_Window.verticalScrollBar().maximum())
            if (len(self.ConsoleMessage) > 17000):
                self.ConsoleMessage += "# Status Truncated\n"    
        if (time.time() - self.lastDataRequestTime) > 1:
            if (self.wifiConnected):
                self.connectionWrite("u5\n") # heartbeat 
                self.lastDataRequestTime = time.time()
                self.info.timerUpdate()
        if self.ui.tabWidget.currentIndex() == 1:
            self.lineSensor.timerUpdate() 
            
        if self.ui.tabWidget.currentIndex() == 4:   
            self.servo.timerUpdate()
        
        if self.ui.tabWidget.currentIndex() == 2:   
            self.irdist.timerUpdate()    
            
        if self.video_recording and time.time() - self.lastToogleTime > 0.5:
            self.lastToogleTime = time.time()
            self.toogleRecLabel()
            
            
            
        
    def guiRC_enable(self):
        self.guiRC_enabled = not self.guiRC_enabled
        if self.guiRC_enabled:
            self.ConsoleMessage += "*GUI Remote control enabled\n"
            self.ConsoleMessagePush = True
        else:    
            self.ConsoleMessage += "*GUI Remote control disabled\n"
            self.ConsoleMessagePush = True    
        
    def guiRC_fwd(self):
        speed = self.ui.manualGuiSpeed_Cmd.text()
        if self.guiRC_enabled and speed != "" and float(speed): 
            MESSAGE = bytes('rc=1 '+speed+' 0\n', 'utf-8')
            self.socket.send(MESSAGE)
        
    def guiRC_rev(self):
        speed = self.ui.manualGuiSpeed_Cmd.text()
        if self.guiRC_enabled and speed != "" and float(speed):
            rev = float(speed)*(-1)
            MESSAGE = bytes('rc=1 '+ str(rev) +' 0\n', 'utf-8')
            self.socket.send(MESSAGE)  
        
    def guiRC_right(self):
        speed = self.ui.manualGuiSpeed_Cmd.text()
        if self.guiRC_enabled and speed != "" and float(speed):
            rev = float(speed)*(-1)
            MESSAGE = bytes('rc=1 0 '+ str(rev) +'\n', 'utf-8')
            self.socket.send(MESSAGE)  
        
    def guiRC_left(self):
        speed = self.ui.manualGuiSpeed_Cmd.text()
        if self.guiRC_enabled and speed != "" and float(speed):
            MESSAGE = bytes('rc=1 0 '+ speed +'\n', 'utf-8')
            self.socket.send(MESSAGE) 
            
    def guiRC_idle(self):
        if self.guiRC_enabled:    
            MESSAGE = b'rc=1 0 0\n'
            self.socket.send(MESSAGE)                
                              
    def keyboardRC_enable(self):
        self.keyboardRC_enabled = not self.keyboardRC_enabled
        if self.keyboardRC_enabled:
            self.ConsoleMessage += "*Keyboard Remote control enabled\n"
            self.ConsoleMessagePush = True
        else:    
            self.ConsoleMessage += "*Keyboard Remote control disabled\n"
            self.ConsoleMessagePush = True
        self.keyboardRC()
        
    def keyboardRC(self):
        speed = self.ui.manualGuiSpeed_Cmd.text()
        if self.keyboardRC_enabled and speed != "" and float(speed):
            if keyboard.is_pressed('up arrow'):  # if key 'up' is pressed 
                MESSAGE = bytes('rc=1 '+speed+' 0\n', 'utf-8')
                self.socket.send(MESSAGE)
            elif keyboard.is_pressed('down arrow'):  # if key 'down' is pressed
                rev = float(speed)*(-1)
                MESSAGE = bytes('rc=1 '+ str(rev) +' 0\n', 'utf-8')
                self.socket.send(MESSAGE)  
            elif keyboard.is_pressed('left arrow'):  # if key 'left' is pressed
                MESSAGE = bytes('rc=1 0 '+ speed +'\n', 'utf-8')
                self.socket.send(MESSAGE)
            elif keyboard.is_pressed('right arrow'):  # if key 'right' is pressed
                rev = float(speed)*(-1)
                MESSAGE = bytes('rc=1 0 '+ str(rev) +'\n', 'utf-8')
                self.socket.send(MESSAGE) 
            elif keyboard.is_pressed('esc'):  # if key 'esc' is pressed 
                self.ui.enableKeyboardRC_checkBox.setChecked(False) 
                self.keyboardRC_enable()
            else:
                MESSAGE = b'rc=1 0 0\n'
                self.socket.send(MESSAGE)
                    
                                                 
    def readReply(self):
        c = '\0'
        self.socket.settimeout(1)
        while (not self.stop.is_set()):
            if self.wifiConnected:
                n = 0
                if (c == '\n'):
                    MESSAGE = ""
                c = '\0'
                try: 
                      while (c != '\n' and self.wifiConnected):  
                            c = self.socket.recv(1).decode()
                            if (len(c) > 0):
                                    if (c >= ' ' or c == '\n'):
                                        MESSAGE = MESSAGE + str(c)
                      n = len(MESSAGE)
                except:
                      time.sleep(0.01)
                if (n > 0):
                    self.decodeCommand(MESSAGE, n)
            else:
                  time.sleep(0.1)
        pass           

    def decodeCommand(self, MESSAGE, n):
        if n > 0:
            if (self.ui.RX_checkBox.isChecked()):
                self.ConsoleMessage += "(rx)-> " + str(MESSAGE)
                self.ConsoleMessagePush = True
        if n > 1:
            gg = MESSAGE.split()
            
            if gg[0] == "hbt": 
                dog = 1
                #print(gg, 'hbt')
            elif self.info.readData(gg):
                pass
            elif self.lineSensor.readData(gg):
                pass
            elif self.servo.readData(gg): 
                pass
            if self.irdist.readData(gg): 
                pass


           
  
    def connectClient(self):
        if not self.wifiConnected:
            IP = str('192.168.43.101') # Connect automatically to this address
            #IP = str(self.ui.IP_Adress_field.text()) # Manual IP adress input
            # Getting address info for the connection
            for addressInfo in socket.getaddrinfo(IP, 24001, socket.AF_UNSPEC, socket.SOCK_STREAM): 
                print("Socket Info " + str(addressInfo)) # Printing the whole adress info 
                AddresFamily = addressInfo[0] # Extracting the Adress Family type
                print("# Adress Family ", AddresFamily)
                SocketType = addressInfo[1] # Extracting the Socket Type
                print("# Socket Type " , SocketType)
                IP = addressInfo[4] # both IP and port number
                print("# IP adress and port ", IP)
            try:      
                self.socket = socket.socket(AddresFamily, SocketType, 0) # Making a actual networ socket with necessary adress parameters.
                self.socket.settimeout(5)
            except OSError as msg:
                print("# Network Connection timeout - retry")
            try:  
                self.ConsoleMessage += "Connecting to Client..."
                self.ConsoleMessagePush = True
                print("Connecting to Client...")
                self.socket.connect(IP) # Connecting to the adress
                self.wifiConnected = True
                self.setConnectFrameColor(self.ui.IP_Connect_frame,QtGui.QColor(0,255,0,255))
                self.ui.IP_NetworkStatus_label.setText('Connected')
                print("Connected")
                self.ConsoleMessage += "Network is Connected\n"
                self.ConsoleMessage += "Socket Info " + str(addressInfo) + "\n"
                self.ConsoleMessagePush = True
            except OSError as msg:
                self.socket.close() # closing the connection if faulted
                self.ui.IP_NetworkStatus_label.setText('Faulted')
                        
    def disconnectClient(self):
        if self.wifiConnected:
            print("Disconnecting...")
            self.ui.IP_NetworkStatus_label.setText('Disconnecting')
            self.wifiConnected = False
            self.video_StopRecording()
            self.socket.shutdown(2)
            self.socket.close()
            self.setConnectFrameColor(self.ui.IP_Connect_frame,QtGui.QColor(200,0,0,255))
            self.ui.IP_NetworkStatus_label.setText('Disconnected')
            print("Disconnected")
            self.ConsoleMessage += "Network is Disconnected\n"
            self.ConsoleMessagePush = True
    
    def connectionWrite(self,MESSAGE):
        MESSAGE = bytes(MESSAGE, 'utf-8')
        if self.wifiConnected:
            self.socket.send(MESSAGE)
        if (self.wifiConnected):
            if (self.ui.TX_checkBox.isChecked()):
                self.ConsoleMessage += "(tx)->"+ str(MESSAGE) +'\n'
                self.ConsoleMessagePush = True
        else:
            self.ConsoleMessage += "Network not connected, could not send message -" + str(MESSAGE)
        pass
        
        
    def sendCmd(self):
        MESSAGE = self.ui.TextCmd_field.text()+'\n'
        self.connectionWrite(MESSAGE)
       
    def ConsoleClear(self):
        self.ConsoleMessage = ""
        self.ConsoleMessagePush = True
        
    def ConsoleHelp(self):
        self.connectionWrite("help\n")
        self.ConsoleMessagePush = True
   

# RUN APPLICATION
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = GUIMainWindow()
    MainWindow.show()
    sys.exit(app.exec_())
    
    

