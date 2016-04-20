from __future__ import division
import numpy as np
import cv2
import time

from airhockey.hardware.drive import Drive
from airhockey.vision.tracker import Tracker
from controller import NN_controller


drive = Drive('/dev/ttyACM0')
controller = NN_controller(kp=[0.3, 0.1], kd=[0.001, 0.001], kv=0, kw=0, N=1)
controller.learn = True


cam = cv2.VideoCapture(1)  # for visualizing
Hinv = np.array([[  8.65555858e+02,  -1.39602291e+01,  -5.58948971e+05],
				 [ -2.13800520e+00,  -8.85189904e+02,   2.17564523e+04],
				 [ -3.86023473e-03,  -1.82342876e-02,  -6.43637551e+02]])
tracker = Tracker(cam, Hinv=Hinv)


first_time = time.time()
last_time = time.time()
last_odom = np.zeros(4)
last_u = np.zeros(2)

killed = False
area_limits = [50, 500, 50, 450]  # [xmin, xmax, ymin, ymax]

# press 1 for waypoints[0], press 2 for waypoints[1]... or choice == 8 is circle
choice = 0
waypoints = (np.array([100, 150]), np.array([100, 375]), np.array([300, 150]), np.array([300, 375]))
amp = 150  # circle amplitude
center = np.array([300, 300])  # circle center
T = 5  # circle period


print("PRESS ESCAPE TO QUIT")

while True:

	odom, timestamp = drive.get_odom()
	t = timestamp - first_time
	dt = timestamp - last_time
	last_time = timestamp

	tracker.get_target()
	# if not tracker.target_found:
	# 	print("Puck not found!")

	if choice+1 == 9:
		w = 2*np.pi/T
		drive.odom_shift = center + [amp, 0]
		pref = center + amp*np.array([np.cos(w*t), np.sin(w*t)])
		vref = w*amp*np.array([-np.sin(w*t), np.cos(w*t)])
		qref = np.concatenate((pref, vref))
		waypoint = pref
		if t < 0.2:
			killed = True
			print('not started yet')
		else:
			killed = False
	else:
		killed = False
		waypoint = waypoints[choice]
		qref = np.concatenate((waypoint, np.zeros(2)))

	u = controller.get_effort(odom, qref, dt)

	# print dt
	# if not np.allclose(odom, last_odom):
		# print("odom:")
		# print np.round(odom, 1)
		# last_odom = odom
		# print("err:")
		# print np.round(qref - odom)
		# print("u:")
		# print np.round(u, 1)
		# last_u = u
		# print("\n")

	if odom[0] < area_limits[0] or odom[0] > area_limits[1] or odom[1] < area_limits[2] or odom[1] > area_limits[3]:
		drive.set_effort(np.array([0, 0]))
		killed = True
		print "killed"
	elif not killed:
		drive.set_effort(u)
		# print "safety is on"
		pass

	homog = tracker.H.dot(np.concatenate((odom[:2], [1])))
	p_pix = np.array([homog[0]/homog[2], homog[1]/homog[2]])

	homog_waypoint = tracker.H.dot(np.concatenate((waypoint[:2], [1])))
	waypoint_pix = np.array([homog_waypoint[0]/homog_waypoint[2], homog_waypoint[1]/homog_waypoint[2]])

	# paddle state
	cv2.circle(tracker.frame, (int(p_pix[0]), int(p_pix[1])), 5, (255, 0, 0), -1)
	cv2.line(tracker.frame, (int(p_pix[0]), int(p_pix[1])), (int(p_pix[0] - 0.25*odom[2]), int(p_pix[1] + 0.25*odom[3])), (255, 0, 0), 5)

	# waypoint state
	cv2.circle(tracker.frame, (int(waypoint_pix[0]), int(waypoint_pix[1])), 5, (0, 255, 0), -1)
	if choice+1 == 9:
		cv2.line(tracker.frame, (int(waypoint_pix[0]), int(waypoint_pix[1])), (int(waypoint_pix[0] - 0.25*vref[0]), int(waypoint_pix[1] + 0.25*vref[1])), (0, 255, 0), 2)

	# controller efforts
	cv2.line(tracker.frame, (int(p_pix[0]), int(p_pix[1])), (int(p_pix[0] - 0.75*u[0]), int(p_pix[1] + 0.75*u[1])), (0, 165, 255), 2)
	cv2.line(tracker.frame, (int(p_pix[0]), int(p_pix[1])), (int(p_pix[0] - 0.75*controller.y[0]), int(p_pix[1] + 0.75*controller.y[1])), (255, 0, 255), 2)

	cv2.imshow('frame', tracker.frame)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
	elif chr(k) in ['1', '2', '3', '4']:
		choice = int(chr(k)) - 1
		print("Goal #{}".format(choice + 1))

cv2.destroyAllWindows()
drive.kill()
