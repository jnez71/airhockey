"""
Basic Neural Network Controller

A single layer neural network (NN) is used to generate a
feedforward term for a typical state-feedback (PD) controller.
The system is assumed to have error dynamics of the form,

edot = f(x) - u,      i.e. control-affine, holonomic, and time-invariant

The input to the NN is the full state x, and the output is
the estimated system dynamics, f(x). Thus choose,

u = e + NN      ==>      edot = -e + epsilon

where epsilon is the current estimation error of the NN.

The NN weights are updated to minimize e, not epsilon.
This guarantees Lyapunov stability during the learning.
Hyperbolic tangent is used for the NN basis function.

The minimum size of epsilon (ball of convergence) depends
on the structure of the NN itself (size and basis functions).

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import numpy.linalg as npl

################################################# PRIMARY CLASS

class NN_controller:

	def __init__(self, kp, kd, kv, kw, N, u_limit=[100, 100], nn_limit=[100, 100]):
		"""
		kp:  Proportional feedback gains.
		kd:  Derivative feedback gains.
		kv:  Input-side learning gain (scalar).
		kw:  Output-side learning gain (scalar).
		N:   Number of neurons.
		u_limit: Limit on total output effort.
		nn_limit: Limit on NN component feedforward effort.

		The user must actively set self.learn = True to
		have the NN start learning.

		"""
		self.nstates = 2 * len(kp)
		self.ncontrols = len(kp)
		self.nsigs = N

		self.sig = lambda x: np.concatenate(([1], np.tanh(x)))
		self.sigp = lambda x: np.tile(1/(np.cosh(x)**2), (self.nsigs+1, 1))

		self.set_gains(kp, kd, kv, kw)
		self.u_limit = np.array(u_limit, dtype=np.float32)
		self.nn_limit = np.array(nn_limit, dtype=np.float32)

		self.V = np.zeros((self.nstates+1, self.nsigs))
		self.W = np.zeros((self.nsigs+1, self.ncontrols))
		self.y = np.zeros(self.ncontrols)

		self.saturated = False
		self.learn = False

########################

	def set_gains(self, kp, kd, kv, kw):
		self.kp = np.array(kp, dtype=np.float32)
		self.kd = np.array(kd, dtype=np.float32)
		self.kr = self.kp / self.kd  # don't let kd be zero
		self.kv = np.array(kv, dtype=np.float32)
		self.kw = np.array(kw, dtype=np.float32)

########################

	def get_effort(self, q, qref, dt):
		"""
		Returns vector of efforts given a current
		state q and a desired state qref. Updates
		learning over a timestep dt, which should
		be the time since called last.

		"""
		# Tracking errors
		E = qref[:self.ncontrols] - q[:self.ncontrols]
		Edot = qref[self.ncontrols:] - q[self.ncontrols:]
		r = self.kr*E + Edot

		# Control law
		u = self.kp*E + self.kd*Edot + self.y

		# Adapt NN
		if self.learn and not self.saturated:
			x = np.concatenate(([1], q))
			VTx = self.V.T.dot(x)
			Wdot = self.kw.dot(np.outer(self.sig(VTx), r))
			Vdot = self.kv.dot(np.outer(x, r).dot(self.W.T).dot(self.sigp(VTx)))
			self.W = self.W + Wdot*dt
			self.V = self.V + Vdot*dt
			self.y = np.clip(self.W.T.dot(self.sig(self.V.T.dot(x))), -self.nn_limit, self.nn_limit)

		# Safety saturation of output
		self.saturated = False
		for i, mag in enumerate(abs(u)):
			if mag > self.u_limit[i]:
				u[i] = self.u_limit[i] * np.sign(u[i])
				self.saturated = True

		# Return effort torques
		return u
