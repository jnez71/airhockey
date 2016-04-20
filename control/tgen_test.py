from __future__ import division
import numpy as np
import numpy.linalg as npl

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from tgen import Tgen


player = Tgen()

while True:

	paddle = np.concatenate(([800, 1000]*np.random.rand(2), 200*np.random.rand()*(np.random.rand(2)-[1, 0.5])))
	puck = np.concatenate(([2000, 1000]*np.random.rand(2), 200*np.random.rand()*(np.random.rand(2)-[1, 0.5])))
	xpoints, ypoints = zip(*player.get_qref(paddle, puck))
	xgoals, ygoals = zip(*player.goals)


	fig = plt.figure()
	ax = fig.add_subplot(1, 1, 1)
	ax.set_xlim([-1000, 2000])
	ax.set_ylim([-500, 3000])
	ax.grid(True)

	ax.scatter(paddle[1], paddle[0], color='b', s=50)
	
	ax.scatter(puck[1], puck[0], color='r', s=50)
	ax.plot([puck[1], puck[1]+2*puck[3]], [puck[0], puck[0]+2*puck[2]], 'r', linewidth=3)

	ax.scatter(ypoints[1], xpoints[1], color='k', s=50)
	ax.plot(ypoints, xpoints, 'k')

	ax.scatter(ygoals, xgoals, color='g', s=50)

	area = patches.Rectangle((0, 0), 2*player.central_goal[1], player.central_goal[0], fill=False)
	ax.add_patch(area)

	plt.show()
