"""
Class for visually tracking something.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import numpy.linalg as npl

from collections import deque
import time
import cv2

################################################# MAIN CLASS

class Tracker:

    def __init__(self, cam, Hinv=None, dsf=4, hsv_min=[110, 100, 100], hsv_max=[130, 255, 255], tightness=5, est='avg', L=[3, 2]):
        """
        cam: cv2 capture object of the camera to be used.

        Hinv: Transformation from pixel coordinates to world coordinates.
              If left at default (None) then this tracker will run a
              calibration routine.

        dsf: Downsampling factor (frame is downsampled before processing).

        hsv_*: Thresholds in HSV color space (default blue).

        tightness: Inversely proportional to region of interest,
                   used for filtering the first threshold mask.

        est: Method for state estimation filtering. Can be 'obs'
             (zero-accel Luenberger observer with gains L) or
             'avg' (running average over L points).

        L: [position filtering param, velocity filtering param]
           See est description.

        """
        self.cam = cam
        self.set_thresholds(hsv_min, hsv_max, tightness)
        self.dsf = dsf

        self.frame = None
        self.target_found = False
        self.last_centroid = np.zeros(2)
        
        self.last_position = np.zeros(2)
        self.last_time = time.time()
        self.L = np.array(L)

        if est == 'avg':
            self.state_est = self.state_est_average
            self.position_stack = deque([np.zeros(2)] * self.L[0])
            self.position_sum = np.zeros(2)
            self.velocity_stack = deque([np.zeros(2)] * self.L[1])
            self.velocity_sum = np.zeros(2)
        elif est == 'obs':
            self.state_est = self.state_est_observer  # L=[15, 10] is good for this
            self.last_velocity = np.zeros(2)
        else:
            raise ValueError("Unsupported state estimation method requested.")

        if np.any(Hinv):
            self.Hinv = Hinv
            self.H = npl.inv(Hinv)
        else:
            self.calibrate()


    def get_target(self):
        """
        Returns array with target state in world ([px, py, vx, vy], timestamp).

        """
        # Get current frame
        ret, self.frame = self.cam.read()

        # Downsample the frame by a factor of dsf and store its size
        dsframe = cv2.resize(self.frame, dsize=None, fx=1/self.dsf, fy=1/self.dsf)
        size = np.shape(dsframe)

        # Convert BGR to HSV color space
        hsv = cv2.cvtColor(dsframe, cv2.COLOR_BGR2HSV)

        # Threshold to produce binary mask
        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        # cv2.imshow('pre mask', mask) # FOR DEBUG

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
        # cv2.imshow('final mask', mask) # FOR DEBUG

        # Recompute weighted mask's centroid
        M = cv2.moments(mask)
        if not np.isclose(M['m00'], 0):
            centroid = self.dsf * np.array([M['m10']/M['m00'], M['m01']/M['m00']])
            self.last_centroid = centroid
            self.target_found = True
        else:
            centroid = self.last_centroid
            self.target_found = False

        # Convert pixel coordinates to world coordinates for position sensed by camera
        homog = self.Hinv.dot(np.concatenate((centroid, [1])))
        p_sen = np.array([homog[0]/homog[2], homog[1]/homog[2]])
        t = time.time()

        # Finite difference for velocity sensed by camera
        dt = t - self.last_time
        v_sen = (p_sen - self.last_position) / dt

        # State estimation to get filtered p and v
        p, v = self.state_est(p_sen, v_sen, dt)
        self.last_position = p
        self.last_time = t
        
        # Draw onto frame for visualization if desired
        cv2.circle(self.frame, (int(centroid[0]), int(centroid[1])), 5, (0, 0, 255), -1)
        cv2.line(self.frame, (int(centroid[0]), int(centroid[1])),
                 (int(centroid[0] + 0.5*v[0]), int(centroid[1] + 0.5*v[1])), (0, 0, 255), 5)

        # Return vector of full target state, coupled with a timestamp
        return (np.concatenate((p, v)), t)


    def state_est_average(self, p_sen, v_sen, dt):
        """
        Uses history of p and v values to average
        out the new p and v over L points. The average
        is weighted more towards p_sen and v_sen. Note
        dt is only used for observer function option.

        """
        self.position_stack.append(p_sen)
        self.position_sum = (self.position_sum - self.position_stack.popleft()) + p_sen
        p = (self.position_sum + self.L[0]*p_sen) / (2*self.L[0])

        self.velocity_stack.append(v_sen)
        self.velocity_sum = (self.velocity_sum - self.velocity_stack.popleft()) + v_sen
        v = (self.velocity_sum + self.L[1]*v_sen) / (2*self.L[1])

        return (p, v)


    def state_est_observer(self, p_sen, v_sen, dt):
        """
        Zero-accel Luenberger observer to use sensor
        estimates (camera-determined p_est and v_est)
        to define the rate of change of the final
        states p and v (which are returned). If there
        were an IMU on the target, you could use that
        for accel prediction and it would be REALLY good.
        
        """
        p_expected = self.last_position + self.last_velocity * dt
        pdot = self.last_velocity + self.L[0] * (p_sen - p_expected)
        p = self.last_position + pdot*dt

        vdot = self.L[1] * (v_sen - self.last_velocity)
        v = self.last_velocity + vdot*dt
        self.last_velocity = v

        return (p, v)


    def set_thresholds(self, hsv_min, hsv_max, tightness):
        """
        Sets minimum and maximum thresholds in [hue, saturation, value]
        color space and sets the tightness (fraction of image size) for
        computing the radius of interest around the preliminary centroid.
        This function is provided in case you want to do dynamic thresholding.

        """
        self.hsv_min = np.array(hsv_min)
        self.hsv_max = np.array(hsv_max)
        self.tightness = tightness


    def calibrate(self, fid_hue=[1, 40], xy=[5, 3], xY=[5, 16], Xy=[21, 3], XY=[21, 16]):
        """
        Four-point planar homography calibration.
        Sets transformation from pixels to world (Hinv)
        and its inverse (H) as a tuple.
        Takes the fiducial marker color (hue limits) and
        locations of the four markers in world where lowercase
        is min and uppercase is max.
        xy == bottom-left (quadrant 3)
        xY == top-left (quadrant 2)
        Xy == bottom-right (quadrant 4)
        XY == top-right (quadrant 1)

        """
        print("Running calibration routine...")
        print("Press 'y' to accept, or anything else to try again.")
        
        while True:

            # Get a frame
            ret, frame = self.cam.read()
            # frame = cv2.imread('calibration_test_img1.jpg')  # FOR DEBUG

            # Determine size of frame and location of central axes
            size = np.shape(frame)
            axis = [int(size[0]/2), int(size[1]/2)]

            # Convert BGR to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Threshold to produce binary mask, default orange/red
            calib_hsv_min=np.array([fid_hue[0], 75, 75])
            calib_hsv_max=np.array([fid_hue[1], 255, 255])
            mask = cv2.inRange(hsv, calib_hsv_min, calib_hsv_max)

            # Divide up mask into quadrants
            mask_tr = mask[:axis[0], axis[1]:]
            mask_tl = mask[:axis[0], :axis[1]]
            mask_bl = mask[axis[0]:, :axis[1]]
            mask_br = mask[axis[0]:, axis[1]:]

            # Iterate through quadrants to find centroids
            x_pix = np.zeros(4)
            y_pix = np.zeros(4)
            for (i, img) in enumerate([mask_tr, mask_tl, mask_bl, mask_br]):
                M = cv2.moments(img)
                if not np.isclose(M['m00'], 0):
                    x_pix[i], y_pix[i] = np.array([M['m10']/M['m00'], M['m01']/M['m00']])
                else:
                    print("Warning! Can't see fiducial in quadrant {}.".format(i+1))
                    x_pix[i], y_pix[i] = (0, 0)

            # Shift relative centroids back to full frame centroids
            x_pix[0] = x_pix[0] + axis[1]
            y_pix[2] = y_pix[2] + axis[0]
            x_pix[3] = x_pix[3] + axis[1]
            y_pix[3] = y_pix[3] + axis[0]

            # Homography hootenany, Ah = 0
            A = np.array([
                          [xy[0], xy[1], 1,    0,     0,  0, -xy[0]*x_pix[2], -xy[1]*x_pix[2], -x_pix[2]],
                          [    0,     0, 0, xy[0], xy[1], 1, -xy[0]*y_pix[2], -xy[1]*y_pix[2], -y_pix[2]],
                          [xY[0], xY[1], 1,    0,     0,  0, -xY[0]*x_pix[1], -xY[1]*x_pix[1], -x_pix[1]],
                          [    0,     0, 0, xY[0], xY[1], 1, -xY[0]*y_pix[1], -xY[1]*y_pix[1], -y_pix[1]],
                          [Xy[0], Xy[1], 1,    0,     0,  0, -Xy[0]*x_pix[3], -Xy[1]*x_pix[3], -x_pix[3]],
                          [    0,     0, 0, Xy[0], Xy[1], 1, -Xy[0]*y_pix[3], -Xy[1]*y_pix[3], -y_pix[3]],
                          [XY[0], XY[1], 1,    0,     0,  0, -XY[0]*x_pix[0], -XY[1]*x_pix[0], -x_pix[0]],
                          [    0,     0, 0, XY[0], XY[1], 1, -XY[0]*y_pix[0], -XY[1]*y_pix[0], -y_pix[0]]
                        ])
            U, s, V = npl.svd(A.T.dot(A))
            h = U[:, -1]
            H = np.array([
                          [h[0], h[1], h[2]],
                          [h[3], h[4], h[5]],
                          [h[6], h[7], h[8]]
                        ])
            Hinv = npl.inv(H)

            # Draw candidate
            cv2.circle(frame, (int(x_pix[0]), int(y_pix[0])), 5, (0, 0, 255), -1)
            cv2.circle(frame, (int(x_pix[1]), int(y_pix[1])), 5, (0, 255, 0), -1)
            cv2.circle(frame, (int(x_pix[2]), int(y_pix[2])), 5, (255, 0, 0), -1)
            cv2.circle(frame, (int(x_pix[3]), int(y_pix[3])), 5, (0, 0, 0), -1)
            cv2.imshow('Calibration Candidate', frame)

            # Get user acceptance
            choice = cv2.waitKey(delay=-1)
            cv2.destroyWindow('Calibration Candidate')
            if choice == ord('y'):
                print("Hinv =")
                print(Hinv)
                self.Hinv = Hinv
                self.H = H
                break
            else:
                print("How about now?")
