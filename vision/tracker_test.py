from __future__ import division
import numpy as np
import cv2

from tracker import Tracker


cam = cv2.VideoCapture(0)
puck = Tracker(cam, Hinv=np.eye(3))
# puck = Tracker(cam, est='avg', L=[3, 2])


last_puck_state = [0, 0, 0, 0]
print("Press escape to quit!")

while(1):

	puck_state, puck_timestamp = puck.get_target()

	# if not np.allclose(np.round(puck_state, 1), np.round(last_puck_state, 1)):
	# 	last_puck_state = puck_state
	# 	print np.round(puck_state, 1)
	
	cv2.imshow('puck tracker frame', puck.frame)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()


# # Test image calibration test
# Hinv = np.array([[ -1.81797648e+01,  -1.69905902e+00,   8.64370914e+02],
# 				 [ -5.47630694e-02,   1.85444219e+01,  -3.91832267e+03],
#  				 [  9.74296532e-02,  -1.97548765e-01,  -1.74684244e+02]])

# frame = cv2.imread('calibration_test_img1.jpg')
# hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# test_hsv_min=np.array([110, 100, 100])
# test_hsv_max=np.array([130, 255, 255])
# mask = cv2.inRange(hsv, test_hsv_min, test_hsv_max)
# M = cv2.moments(mask)
# centroid = np.array([M['m10']/M['m00'], M['m01']/M['m00']])
# cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
# cv2.imshow('test cal', frame)
# print("Expect:\n[15, 7]")
# print("Got:")
# homog = Hinv.dot(np.concatenate((centroid, [1])))
# print np.round(np.array([homog[0]/homog[2], homog[1]/homog[2]]), 2)
# print("Press anything to quit.")
# cv2.waitKey(delay=-1)
