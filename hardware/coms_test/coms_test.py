from __future__ import division
import time
import serial
import struct

com = serial.Serial('/dev/ttyACM1', 9600)
time.sleep(1)

com.write('a')
print(com.read())
