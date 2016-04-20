"""
Class for communicating to an arduino running motordriver.ino.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
from collections import deque
import time
import serial
import struct

################################################# MAIN CLASS

class Drive():

	def __init__(self, port, baud=9600, odom_scale=0.03556, odom_shift=[112, 149], dpoints=3):
		"""
		Set-up serial communication and internal variables.
		Also takes scale (conversion from encoder tics to table mm),
		shift (initial gantry position), and the number of points
		to average the velocity estimate over.

		"""
		self.com = serial.Serial(port, baud)
		time.sleep(1)  # critical for uP to wake-up
		self.__enc_request = '1'
		self.__cmd_request = '2'
		self.killed = False
		
		self.odom_scale = float(odom_scale)
		self.odom_shift = np.array(odom_shift, dtype=np.float32)
		self.last_position = self.odom_shift
		self.last_time = time.time()
		
		self.dpoints = int(dpoints)
		self.velocity_stack = deque([np.zeros(2)] * self.dpoints)
		self.velocity_sum = np.zeros(2)


	def __del__(self):
		"""
		When the MotorDriver object is destroyed,
		kill both motors before terminating.
		"""
		self.kill()


	def get_odom(self):
		"""
		Returns an array of gantry odometry in mm and mm/s,
		packaged with current time as ([px, py, vx, vy], timestamp).

		"""
		# Reset serial buffer pointer
		self.com.flushInput()
		# Request angular odometry
		self.com.write(self.__enc_request)
		# Receive angular odometry
		odom_x, odom_y = struct.unpack('<ll', self.com.read(8))
		# Compute gantry position
		p = self.odom_scale*np.array([odom_x, odom_y]) + self.odom_shift
		t = time.time()
		# Finite difference for velocity estimate
		v = (p - self.last_position) / (t - self.last_time)
		self.last_position = p
		self.last_time = t
		# Running average over dpoints number of recent velocities weighted
		# towards current (ideally will be replaced with kalman filter)
		self.velocity_stack.append(v)
		self.velocity_sum = (self.velocity_sum - self.velocity_stack.popleft()) + v
		v_avg = (self.velocity_sum + self.dpoints*v) / (2*self.dpoints)
		# Return vector of full state odom, coupled with a timestamp
		return (np.concatenate((p, v_avg)), t)


	def set_effort(self, effort):
		"""
		Takes an array-like of two motor percentages -100 to 100.

		"""
		# Bound efforts
		eff = np.clip(effort, -100, 100)
		# Request to give command
		self.com.write(self.__cmd_request)
		# Pack as string and send out command
		packet = str(int(eff[0])) + ' ' + str(int(eff[1]))
		self.com.write(packet)
		# Wait for acknowledgement
		self.com.read(1)


	def kill(self):
		"""
		Soft-kills both motors.

		"""
		self.com.write(self.__cmd_request)
		time.sleep(0.25)
		self.com.write('0 0')
