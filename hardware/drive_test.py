from __future__ import division
import numpy as np
import time
from drive import Drive

drive = Drive()

last_odom = [None, None, None, None]
ID = 1

while True:

	# ODOM TEST:

	odom, timestamp = drive.get_odom()

	if not np.allclose(odom, last_odom):
		last_odom = odom
		print np.round(odom, 2)

	time.sleep(10/1000)


	# MOTOR TEST:

	# val = 30
	# rest = 1

	# print "forward"
	# drive.set_effort([val, val])
	# time.sleep(rest)

	# print "rest"
	# drive.set_effort([0, 0])
	# time.sleep(rest)

	# print "reverse"
	# drive.set_effort([-val, -val])
	# time.sleep(rest)

	# print "rest"
	# drive.set_effort([0, 0])
	# time.sleep(rest)

	# # found 30 to be the break-free effort
