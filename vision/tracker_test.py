from __future__ import division
import cv2
from tracker import Tracker

puck = Tracker()

while(1):

	position_stamped = puck.get_position()

	cv2.circle(puck.frame, (int(puck.last_centroid[0]), int(puck.last_centroid[1])), 5, (0,0,255), -1)

	for (i, val) in enumerate(puck.position_stack):
		cv2.circle(puck.frame, (int(val[0][0]), int(val[0][1])), 5, (0,0,0), -1)

	cv2.imshow('frame', puck.frame)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()
