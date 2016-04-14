"""
Class for communicating to an arduino running drive.ino.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import time
import serial
import struct

################################################# MAIN CLASS

class MotorDriver:

	def __init__(self, port='/dev/ttyACM1', baud=9600, odom_scale=1.0):
		"""
		Set-up serial communication.
		Set scaling from motor rotation to table motion.

		"""
		self.com = serial.Serial(port, baud)
		time.sleep(1)  # critical for uP to wake-up

		self.__enc_request = '1'
		self.__cmd_request = '2'

		self.killed = False
		self.odom_scale = float(odom_scale)


	def __del__(self):
		"""
		When the MotorDriver object is destroyed,
		kill both motors before terminating.
		"""
		self.kill()


	def get_odom(self):
		"""
		Returns an array of gantry odometry in meters,
		packaged with current time as ([x, y], timestamp).

		"""
		# Reset serial buffer pointer
		self.com.flushInput()
		# Request odometry
		self.com.write(self.__enc_request)
		# Receive odometry
		odom_x, odom_y = np.round(struct.unpack('<ff', self.com.read(8)), 2) * self.odom_scale
		return (np.array([odom_x, odom_y]), time.time())


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
