"""
Module for interfacing with camera.

All tracker objects should get frames from
the same camera and not try to establish
more than one camera connection. Thus all
tracker objects contact the camera through
the cam defined in this module. This module
also performs the calibration for said camera.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import numpy.linalg as npl
import cv2

################################################# MAIN

# Set-up camera
cam = cv2.VideoCapture(1)


# Define calibration routine
def calibrate(bl=[0, 0], br=[0, 0], fl=[0, 0], fr=[0, 0]):
    """
    Calibration...

    """
    global cam
    return np.eye(3)


# Run calibration routine
H = calibrate()
