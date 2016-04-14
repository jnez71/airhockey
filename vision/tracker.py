"""
Class for tracking something in the field of view of
the camera defined in the vision module.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import numpy.linalg as npl
from collections import deque

import time
import cv2
import vision

################################################# MAIN CLASS

class Tracker:

    def __init__(self, hsv_min=[110, 100, 100], hsv_max=[130, 255, 255], tightness=5, dsf=4, vpoints=5):
        """
        Takes hsv thresholds for the target to be tracked, as well as tightness for image
        filtering, the downsampling factor for image processing, and the number of
        positions over which the velocity of the thing should be computed.

        """
        self.set_thresholds(hsv_min, hsv_max, tightness)
        self.dsf = dsf

        self.frame = None
        self.last_centroid = np.zeros(2)
        self.position_stack = deque([([0, 0], 0)] * vpoints)


    def get_position(self):
        """
        Returns (array with target position in world [x, y], timestamp).

        """
        # Get current frame
        ret, self.frame = vision.cam.read()

        # Downsample the frame by a factor of ds and store its size
        dsframe = cv2.resize(self.frame, dsize=None, fx=1/self.dsf, fy=1/self.dsf)
        size = np.shape(dsframe)

        # Convert BGR to HSV color space
        hsv = cv2.cvtColor(dsframe, cv2.COLOR_BGR2HSV)

        # Threshold to produce binary mask
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        cv2.imshow('pre mask', mask) # FOR DEBUG

        # Compute preliminary centroid as centroid of whole image
        M = cv2.moments(mask)
        if not np.isclose(M['m00'], 0):
            pre_centroid = np.array([M['m10']/M['m00'], M['m01']/M['m00']])
        else:
            pre_centroid = self.last_centroid

        # Compute region of interest around preliminary centroid
        radius = max(size) / self.tightness

        # Re-threshold mask based on if pixels are in region of interest
        for x in xrange(size[1]):
            for y in xrange(size[0]):
                # For each true pixel
                if mask[y, x]:
                    # Compute distance to preliminary centroid
                    dist = npl.norm([x, y] - pre_centroid)
                    # If out of region of interest
                    if dist > radius:
                        # Revoke trueness
                        mask[y, x] = 0

        # Recompute weighted mask's centroid
        M = cv2.moments(mask)
        if not np.isclose(M['m00'], 0):
            centroid = self.dsf * np.array([M['m10']/M['m00'], M['m01']/M['m00']])
            self.last_centroid = centroid
        else:
            centroid = self.last_centroid
        cv2.imshow('final mask', mask) # FOR DEBUG

        # Convert pixel coordinates to world coordinates, package with timestamp
        homog = vision.H.dot(np.concatenate((centroid, [1])))
        position_stamped = (np.array([homog[0]/homog[2], homog[1]/homog[2]]), time.time())

        # Push-pop this position into the stack, and return
        self.position_stack.append(position_stamped)
        self.position_stack.popleft()
        return position_stamped


    def get_velocity(self):
        """
        Returns (array with target velocity in world [vx, vy], timestamp).

        """
        pass


    def set_thresholds(self, hsv_min, hsv_max, tightness):
        """
        Sets minimum and maximum thresholds in [hue, saturation, value]
        color space and sets the tightness (fraction of image size) for
        computing the radius of interest around the preliminary centroid.

        """
        self.hsv_min = np.array(hsv_min)
        self.hsv_max = np.array(hsv_max)
        self.tightness = tightness
