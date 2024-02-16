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

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        cap = cv2.VideoCapture('http://cleo.local:8080/?action=stream') # maybe replace with something else?
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                self.change_pixmap_signal.emit(cv_img) #cv_img    
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

class VideoRecordingThread(QThread): 
    change_pixmap_signal = pyqtSignal(np.ndarray)
    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        cap = cv2.VideoCapture('http://cleo.local:8080/?action=stream') # maybe replace with something else?
        frame_width = int(cap.get(3))
        frame_height = int(cap.get(4))
        out = cv2.VideoWriter('PiVideo_1.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                out.write(cv_img)  
        cap.release()
        out.release()

    def stop(self):
        self._run_flag = False
        self.wait()        
    