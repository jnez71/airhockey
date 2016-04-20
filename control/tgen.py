"""
NOT DONE YET

Class for receding horizon player AI.
Use to aquire the next desired paddle
state based on curret paddle state
and puck state.

"""

################################################# DEPENDENCIES

from __future__ import division
import numpy as np
import numpy.linalg as npl

################################################# PRIMARY CLASS

class Tgen:

	def __init__(self, step_size=10, travel_speed=500, swoop_factor=0.2, speed_range=[20, 200],
				 limits=[50, 500, 50, 450], home=[100, 500], goal=[2000, 500]):
		"""
		step_size: The position of qref will always be exactly step_size mm
				   ahead of the current puck position.

		travel_speed: The velocity of qref will always have a magnitude of
					  travel_speed along the tangent to the path.

		swoop_factor: The amount that point2 is "pulled back" from the puck
					  position is proportional to the dist2puck and this factor.

		speed_range: Above this range, defend. Inside, think and attack. Below, attack.
					 (At low speeds puck velocity est is too noisy to think).

		limits: Do not ever go to a position outside of [xmin, xmax, ymin, ymax] mm.

		home: Home position, usually right in front of our goal.

		goal: Opponent goal position.

		"""

		self.step_size = step_size  # mm
		self.travel_speed = travel_speed  # mm/s
		self.swoop_factor = swoop_factor  # mm
		self.speed_range = speed_range  # [min, max] mm/s
		self.limits = limits  # [xmin, xmax, ymin, ymax] mm
		self.home = np.array(home, dtype=np.float32)  # [x, y] mm

		self.central_goal = np.array(goal, dtype=np.float32)  # [x, y] mm
		self.left_goal = np.array([goal[0], 3*goal[1]], dtype=np.float32)
		self.right_goal = np.array([goal[0], -goal[1]], dtype=np.float32)
		self.goals = [self.central_goal, self.left_goal, self.right_goal]


	def get_qref(self, paddle, puck, target_lost=False):
		"""
		If puck is moving away from us, go home.

		If puck is behind us, stay put.

		If puck is moving towards us, evaluate situation.

			If puck is moving faster than speed range, block.

			If puck is moving slower than speed range, choose goal_vector = central and goto @.

			If puck is moving within speed range, decide.

				Compute vector from puck to each goal.
				
				Determine which goal vector is most along the puck velocity.
				
				Point 1 = paddle                                               @
				Point 2 = puck - swoop * dist2puck * goal_vector
				Point 3 = puck
				
		Generate spline or piecepath with 3 points.
		
		Evaluate spline or piecepath arclength.
		
		Compute step_size / arclength to find percent along spline.
		
		Compute spline position and tangent at that percent.
		
		If position is outside of limits, return puck state with zero velocity.

		If position is inside of limits, return position and travel_speed * tangent.

		"""
		if target_lost or puck[2] > 0:

			point2 = np.copy(self.home)
			point3 = np.copy(self.home)

			print "go home"

		elif puck[0] < paddle[0]:

			point2 = paddle[:2]
			point3 = paddle[:2]

			print "stay put"

		else:

			puck_speed = npl.norm(puck[2:])

			if puck_speed > self.speed_range[1]:

				point2 = np.array([self.home[0], np.clip(puck[1], self.limits[2], self.limits[3])])
				point3 = np.copy(point2)

				print "defend"

			else:

				if puck_speed < self.speed_range[0]:

					goal_vector = self.central_goal - puck[:2]
					goal_vector = goal_vector / npl.norm(goal_vector)

					print "too slow, attack central"

				else:

					scores = np.zeros(len(self.goals))

					for i, goal in enumerate(self.goals):

						scores[i] = -goal.dot(puck[2:]) / npl.norm(goal)

					winner = np.argmax(scores)

					goal_vector = self.goals[winner] - puck[:2]
					goal_vector = goal_vector / npl.norm(goal_vector)


					if winner == 0:
						print "attack central"
					elif winner == 1:
						print "attack left"
					elif winner == 2:
						print "attack right"


				point2 = puck[:2] - self.swoop_factor * npl.norm(puck[:2] - paddle[:2]) * goal_vector
				point3 = puck[:2]

		print "\n"

		return (paddle[:2], point2, point3)
