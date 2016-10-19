import numpy as np


class Basestation():
	def __init__(self, address, x, y):
		self.address = address
		self.position = np.array([x, y])

	def distanceTo(self, x, y):
		self.distance(np.array([x, y]))

	def distance(self, obj_pos):
		return np.sqrt(self.position * obj_pos)
