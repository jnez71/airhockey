from __future__ import division
import time
from motordriver import MotorDriver

md = MotorDriver()

last_odom = [None, None]
ID = 1

while True:
	odom, timestamp = md.get_odom()

	if last_odom[0] != odom[0] or last_odom[1] != odom[1]:
		last_odom = odom
		print odom

	md.set_effort((odom[ID], odom[ID]))
	time.sleep(10/1000)
