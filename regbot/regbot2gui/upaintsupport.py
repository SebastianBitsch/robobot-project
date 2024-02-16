#!/usr/bin/python
# -*- coding: utf-8 -*-

# "$Rev: 1262 $"

#import sys
#import os
#from PyQt4 import QtGui, QtCore, Qt
from PyQt5 import QtCore, QtGui
#from pyqtgraph.Qt import QtGui #, QtCore




class PaintSupport(QtGui.QWidget):
  def __index__(self, widget):
      super(PaintSupport, self).__init__(self, widget)
  def paintPlus(self, paint, x, y):
      paint.drawLine(QtCore.QPoint(x - 3, y), QtCore.QPoint(x + 3, y))
      paint.drawLine(QtCore.QPoint(x, y - 3), QtCore.QPoint(x, y + 3))
  def paintMinus(self, paint, x, y):
      paint.drawLine(QtCore.QPoint(x - 3, y), QtCore.QPoint(x + 3, y))
  def paintDot(self, paint, x, y):
      paint.setBrush(QtCore.Qt.black)
      paint.drawEllipse(QtCore.QPoint(x, y), 3, 3)
      paint.setBrush(QtCore.Qt.NoBrush)
  def paintArrow(self, paint, x, y, dir, color = QtCore.Qt.black):
      dx1 = -5 # down
      dx2 = +5
      dy1 = -5
      dy2 = -5
      if (dir == 0): # right
        dx1 = -5
        dx2 = -5
        dy1 = -5
        dy2 = +5
      elif (dir == 1): # up
        dx1 = -5
        dx2 = +5
        dy1 = +5
        dy2 = +5
      elif (dir == 2): # left
        dx1 = +5
        dx2 = +5
        dy1 = -5
        dy2 = +5
      elif (dir == 4): # up-right
        dx1 = 0
        dx2 = -7
        dy1 = +7
        dy2 = 0
      pts = [QtCore.QPoint(x,y), 
             QtCore.QPoint(x + dx1,y + dy1),
             QtCore.QPoint(x + dx2,y + dy2)]
      poly = QtGui.QPolygon(pts)
      paint.setBrush(color)
      paint.drawPolygon(poly)
      paint.setBrush(QtCore.Qt.NoBrush)
  pass

# //////////////////////////////////////////////////////////////7
# //////////////////////////////////////////////////////////////7
  
class UsbPaintSpace(PaintSupport):
  def __init__(self, frame, robot):
    self.robot = robot
    print("UsbPaintSpace call init")
    super(UsbPaintSpace, self).__init__(frame)
    self.ui = robot.ui
  def paintEvent(self, QPaintEvent):
    paint = QtGui.QPainter()
    paint.begin(self)
    paint.setPen(QtCore.Qt.darkGreen)
    if (self.robot.isConnected()):
      paint.setBrush(QtCore.Qt.green)
    else:
      paint.setBrush(QtCore.Qt.NoBrush)
    paint.drawRect(1,1, self.ui.frame_usb_connect.width(), self.ui.frame_usb_connect.height())
    paint.setBrush(QtCore.Qt.NoBrush)
    #print("usb frame painted")
    pass
      
class WifiPaintSpace(PaintSupport):
  def __init__(self, frame, robot):
    super(WifiPaintSpace, self).__init__(frame)
    self.robot = robot
    self.ui = robot.ui
  def paintEvent(self, QPaintEvent):
    paint = QtGui.QPainter()
    paint.begin(self)
    paint.setPen(QtCore.Qt.darkBlue)
    if self.robot.wifiConnected and not self.robot.isConnected():
      # green if wifi is connected and USB is not
      paint.setBrush(QtCore.Qt.green)
    else:
      paint.setBrush(QtCore.Qt.NoBrush)
    paint.drawRect(1,1, self.ui.frame_wifi_connect.width(), self.ui.frame_wifi_connect.height())
    paint.setBrush(QtCore.Qt.NoBrush)
    #print("wifi frame painted")
    pass


class ControlPaintSpace(PaintSupport):
  def __init__(self, frame, robot):  
    super(ControlPaintSpace, self).__init__(frame)
    self.robot = robot
    self.ui = robot.ui
  def getPoint(self, frame, side):
    if side==0: # right side mid
      return QtCore.QPoint(frame.x() + frame.width(), frame.y() + frame.height()/2)
    elif side==1: # top midt
      return QtCore.QPoint(frame.x() + frame.width()/2, frame.y())
    elif side==2: # left midt
      return QtCore.QPoint(frame.x(), frame.y() + frame.height()/2)
    else: # buttom midt
      return QtCore.QPoint(frame.x() + frame.width()/2, frame.y() + frame.height())
                          
  def paintEvent(self, QPaintEvent):
    paint = QtGui.QPainter()
    paint.begin(self)
    dtured=QtGui.QColor(0x99, 0, 0)
    dtublue=QtGui.QColor(0x2f, 0x3e, 0xea)
    dtugreen=QtGui.QColor(0, 0x88, 0x35)
    col = dtured
    paint.setBrush(QtCore.Qt.NoBrush)
    pen = QtGui.QPen(col, 1, QtCore.Qt.SolidLine)
    paint.setPen(pen)
    #
    # from turn and forward to motor control
    #p1 = self.getPoint(self.ui.frame_mix_select, 0)
    #p2 = self.getPoint(self.ui.frame_ctrl_motor, 1)
    p1Mix = self.getPoint(self.ui.frame_mix_select, 0)
    p1Fwd = self.getPoint(self.ui.frame_mix_fwd, 0)
    p1MotorT = QtCore.QPoint(self.ui.frame_ctrl_motor.x() + self.ui.frame_ctrl_motor.width()/3, self.ui.frame_ctrl_motor.y());
    p1MotorV = self.getPoint(self.ui.frame_ctrl_motor, 1)
    paint.drawLine(p1Mix, QtCore.QPoint(p1MotorT.x(), p1Mix.y())) # turn to above motor ctrl 1/3
    paint.drawLine(QtCore.QPoint(p1MotorT.x(), p1Mix.y()), p1MotorT) # down to vel Ctrl
    paint.drawLine(p1Fwd, QtCore.QPoint(p1MotorV.x(), p1Fwd.y())) # Forward to above motor ctrl 1/2
    paint.drawLine(QtCore.QPoint(p1MotorV.x(), p1Fwd.y()), p1MotorV) # down to vel Ctrl
    self.paintArrow(paint, p1MotorT.x(), p1MotorT.y(), 3, col)
    self.paintArrow(paint, p1MotorV.x(), p1MotorV.y(), 3, col)
    #
    # from motor control to Motors
    p1 = self.getPoint(self.ui.frame_ctrl_motor, 3)
    p2 = self.getPoint(self.ui.frame_motors, 1)
    paint.drawLine(p1,p2)
    self.paintArrow(paint, p2.x(), p2.y(), 3, col)
    # 
    # from balance to forward
    p2tilt = QtCore.QPoint(self.ui.frame_ctrlBal.x() + self.ui.frame_ctrlBal.width(), self.ui.frame_ctrlBal.y() + self.ui.frame_ctrlBal.height()/2); # balance ctrl
    p1Fwd = self.ui.frame_mix_fwd.x()
    paint.drawLine(p2tilt, QtCore.QPoint(p1Fwd, p2tilt.y())) # from bal to fwd
    self.paintArrow(paint, p1Fwd, p2tilt.y(), 0, col)
    #
    # from Wall distance to Fwd
    p1 = self.getPoint(self.ui.frame_ctrlWallVel, 0)
    p2 = self.getPoint(self.ui.frame_ctrlBalVel, 3)
    p2.setX(p2.x() - self.ui.frame_ctrlBalVel.width()/6)
    p3 = QtCore.QPoint(p2.x(), p1.y())
    paint.drawLine(p1,p3)
    paint.drawLine(p3,p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p2 = self.getPoint(self.ui.frame_ctrl_vel, 1)
    p2.setX(p2.x() - self.ui.frame_ctrl_vel.width()/6)
    paint.drawLine(p3,p2)
    self.paintArrow(paint, p2.x(), p2.y(), 3, col)
    #
    p1MixX = self.ui.frame_mix_select.x()
    #
    # from wall distance to turn
    p1 = self.getPoint(self.ui.frame_ctrlWallTurn, 0)
    p2.setY(p1.y())
    p2.setX(self.ui.frame_turn_radius.x() + self.ui.frame_turn_radius.width()/3)
    p3.setX(p2.x())
    p3.setY(self.ui.frame_turn_radius.y())
    paint.drawLine(p1, p2)
    paint.drawLine(p2, p3)
    self.paintArrow(paint, p3.x(), p3.y(), 3, col)
    #
    # from edge control to turn radius
    p1 = self.getPoint(self.ui.frame_ctrlEdge, 0)
    p2 = self.getPoint(self.ui.frame_turn_radius, 2)
    paint.drawLine(p1, p2) # from line edge to mix
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    # and from radius to turn
    p1 = self.getPoint(self.ui.frame_turn_radius, 0)
    p2 = self.getPoint(self.ui.frame_mix_select, 2)
    p2.setY(p1.y())
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    #
    # from position to velocity
    p1 = self.getPoint(self.ui.frame_ctrlPos, 0)
    p2 = self.getPoint(self.ui.frame_ctrl_vel, 2)
    paint.drawLine(p1, p2) 
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    #
    # from position to velocity
    p1 = self.getPoint(self.ui.frame_ctrl_vel, 0)
    p2 = self.getPoint(self.ui.frame_mix_fwd, 2)
    p2.setY(p1.y())
    paint.drawLine(p1, p2) 
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    #
    # from heading to turn
    p1 = self.getPoint(self.ui.frame_turn, 0)
    p2.setY(p1.y())
    p2.setX(self.ui.frame_turn_radius.x() + self.ui.frame_turn_radius.width()/3)
    p3.setX(p2.x())
    p3.setY(self.ui.frame_turn_radius.y() + self.ui.frame_turn_radius.height())
    paint.drawLine(p1, p2)
    paint.drawLine(p2, p3)
    self.paintArrow(paint, p3.x(), p3.y(), 1, col)
    #
    # from bal-vel to bal
    p1 = self.getPoint(self.ui.frame_ctrlBalVel, 0)
    p2 = self.getPoint(self.ui.frame_ctrlBal, 2)
    paint.drawLine(p1, p2) 
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    #
    # from balance pos to balance vel
    p1 = self.getPoint(self.ui.frame_ctrlBalPos, 0)
    p2 = self.getPoint(self.ui.frame_ctrlBalVel, 2)
    paint.drawLine(p1, p2) 
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    #
    # from mission to the right
    p1MisX = self.ui.frame_mission_ctrl.x() + self.ui.frame_mission_ctrl.width()
    p2 = self.getPoint(self.ui.frame_ctrlBalPos, 2)
    paint.drawLine(QtCore.QPoint(p1MisX, p2.y()), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    p2 = self.getPoint(self.ui.frame_ctrlWallVel, 2)
    paint.drawLine(QtCore.QPoint(p1MisX, p2.y()), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    p2 = self.getPoint(self.ui.frame_ctrlPos, 2)
    paint.drawLine(QtCore.QPoint(p1MisX, p2.y()), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    p2 = self.getPoint(self.ui.frame_ctrlWallTurn, 2)
    paint.drawLine(QtCore.QPoint(p1MisX, p2.y()), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    p2 = self.getPoint(self.ui.frame_ctrlEdge, 2)
    paint.drawLine(QtCore.QPoint(p1MisX, p2.y()), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    p2 = self.getPoint(self.ui.frame_turn, 2)
    paint.drawLine(QtCore.QPoint(p1MisX, p2.y()), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 0, col)
    #
    # up from sensors
    # - to mission
    col = dtublue
    pen.setColor(col)
    paint.setPen(pen)
    p1SenY = self.ui.frame_sensors.y()
    p2 = self.getPoint(self.ui.frame_ctrlPos, 3)
    paint.drawLine(QtCore.QPoint(p2.x(), p1SenY), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p2 = self.getPoint(self.ui.frame_turn, 3)
    paint.drawLine(QtCore.QPoint(p2.x(), p1SenY), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p2 = self.getPoint(self.ui.frame_turn_radius, 3)
    paint.drawLine(QtCore.QPoint(p2.x(), p1SenY), p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    # inter box arrows - upwards
    p1 = self.getPoint(self.ui.frame_ctrlPos, 1)
    p2 = self.getPoint(self.ui.frame_ctrlWallVel, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = self.getPoint(self.ui.frame_ctrlWallVel, 1)
    p2 = self.getPoint(self.ui.frame_ctrlBalPos, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = self.getPoint(self.ui.frame_ctrl_vel, 1)
    p2 = self.getPoint(self.ui.frame_ctrlBalVel, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = self.getPoint(self.ui.frame_turn, 1)
    p2 = self.getPoint(self.ui.frame_ctrlEdge, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = self.getPoint(self.ui.frame_ctrlEdge, 1)
    p2 = self.getPoint(self.ui.frame_ctrlWallTurn, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = self.getPoint(self.ui.frame_ctrlWallTurn, 1)
    p2 = self.getPoint(self.ui.frame_ctrl_vel, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = self.getPoint(self.ui.frame_turn_radius, 1)
    p2 = self.getPoint(self.ui.frame_ctrlBal, 3)
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    p1 = QtCore.QPoint(self.ui.frame_sensors.x() + self.ui.frame_sensors.width(), self.ui.frame_sensors.y())
    p2 = QtCore.QPoint(self.ui.frame_ctrl_motor.x(), self.ui.frame_ctrl_motor.y() + self.ui.frame_ctrl_motor.height())
    paint.drawLine(p1, p2)
    self.paintArrow(paint, p2.x(), p2.y(), 4, col)
    #
    # from actuator to sensor
    p1 = self.getPoint(self.ui.frame_motors, 3)
    p2 = self.getPoint(self.ui.frame_sensors, 3)
    pen.setStyle(QtCore.Qt.DashDotDotLine)
    paint.setPen(pen)
    paint.drawLine(p1, QtCore.QPoint(p1.x(), p1.y() + 20))
    paint.drawLine(QtCore.QPoint(p1.x(), p1.y() + 20), QtCore.QPoint(p2.x(), p1.y() + 20))
    paint.drawLine(QtCore.QPoint(p2.x(), p2.y() + 20), p2)
    pen.setStyle(QtCore.Qt.SolidLine)
    paint.setPen(pen)
    self.paintArrow(paint, p2.x(), p2.y(), 1, col)
    #paint.end()
    pass




