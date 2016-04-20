# Trivial play style, just block and push.
# NOT DONE TESTING

from __future__ import division
import numpy as np
import cv2
import time

from hardware.drive import Drive
from vision.tracker import Tracker
from control.controller import NN_controller

Hinv = np.array([[  8.65555858e+02,  -1.39602291e+01,  -5.58948971e+05],
				 [ -2.13800520e+00,  -8.85189904e+02,   2.17564523e+04],
				 [ -3.86023473e-03,  -1.82342876e-02,  -6.43637551e+02]])

paddle = Drive('/dev/ttyACM0')
controller = NN_controller(kp=[0.3, 0.25], kd=[0.008, 0.008], kv=0.005, kw=0.005, N=3)

cam = cv2.VideoCapture(1)
puck = Tracker(cam, Hinv=Hinv)

attack_threshold = 200  # mm
attack_magnitude = 300  # mm
y_limit = [50, 500]  # mm

first_time = time.time()
last_time = time.time()

print("PRESS ESCAPE TO QUIT")
cv2.destroyAllWindows()

while True:

	# Read in
	q, paddle_timestamp = paddle.get_odom()
	puck_state, puck_timestamp = puck.get_target()

	# Desire to be at [starting x, limited puck y, 0 xvel, puck yvel]
	qref = np.array([paddle.odom_shift[0], np.clip(puck_state[1], y_limit[0], y_limit[1]), 0, 0])

	# # Attack if puck is close
	# if puck_state[2] < 0 and puck_state[0] < paddle.odom_shift[0] + attack_threshold and abs(puck_state[1] - q[1]) < 30:
	# 	controller.set_gains(kp=[0.4, 0.4], kd=[0.01, 0.01], kv=0, kw=0)
	# 	qref[0] = paddle.odom_shift[0] + attack_magnitude
	# 	# print "attacking"
	# # Return softly
	# elif qref[0] < q[0] and puck_state[2] > 0:
	# 	# print "returning softly"
	# 	controller.set_gains(kp=[0.25, 0], kd=[0.01, 0], kv=0, kw=0)
	# # Track normal
	# else:
	# 	# print "tracking"
	# 	controller.set_gains(kp=[0.3, 0.3], kd=[0.008, 0.005], kv=0, kw=0)
	# # print "\n"


	# Apply control
	t = paddle_timestamp - first_time
	dt = paddle_timestamp - last_time
	last_time = paddle_timestamp
	if puck_state[0] > q[0] and t > 1 and puck_state[1] < y_limit[1] and puck_state[1] > y_limit[0]:
		controller.learn = True
		u = controller.get_effort(q, qref, dt)
	else:
		controller.learn = False
		u = [0, 0]
	paddle.set_effort(u)

	# Visualize paddle state (q)
	homog_q = puck.H.dot(np.concatenate((q[:2], [1])))
	q_pix = np.array([homog_q[0]/homog_q[2], homog_q[1]/homog_q[2]])
	cv2.circle(puck.frame, (int(q_pix[0]), int(q_pix[1])), 5, (255, 0, 0), -1)
	cv2.line(puck.frame, (int(q_pix[0]), int(q_pix[1])), (int(q_pix[0] - 0.25*q[2]), int(q_pix[1] + 0.25*q[3])), (255, 0, 0), 5)

	# Visualize qref position
	homog_qref = puck.H.dot(np.concatenate((qref[:2], [1])))
	qref_pix = np.array([homog_qref[0]/homog_qref[2], homog_qref[1]/homog_qref[2]])
	cv2.circle(puck.frame, (int(qref_pix[0]), int(qref_pix[1])), 5, (0, 255, 0), -1)

	# Visualize controller effort
	cv2.line(puck.frame, (int(q_pix[0]), int(q_pix[1])), (int(q_pix[0] - 2*u[0]), int(q_pix[1] + 2*u[1])), (0, 165, 255), 2)
	cv2.line(puck.frame, (int(q_pix[0]), int(q_pix[1])), (int(q_pix[0] - 2*controller.y[0]), int(q_pix[1] + 2*controller.y[1])), (255, 0, 255), 2)

	cv2.imshow('frame', puck.frame)

	# Quit if ESCAPE
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

paddle.kill()
