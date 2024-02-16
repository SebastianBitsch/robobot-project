'''
Documentation, License etc.

@package serialread
'''
import serial
from time import sleep

ser = serial.Serial(port='/dev/ttyACM0', timeout=1)
print("connected to: " + ser.portstr)
count=1
m = 0
try:
  got = ser.readline()
except:
  print("read error")
  pass
while True:
  try:
    got = ser.readline()
    print(str(count) + " " + str(m) + str(': ') + got )
    count = count+1
  except:
    m = m + 1
    sleep(0.01)
    pass
ser.close()