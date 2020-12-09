#!/usr/bin/env python

"""
Turn the given goals to a graph for use in the TSP solving algorithm
"""

import math

class Node:
	def __init__(self, x, y, z, theta):
		self.x = x
		self.y = y
		self.z = z
		self.theta = theta
    
    	def euclidean_distance(self, goal):
		"""
		Method to compute distance from current position to the goal/node being inspected
		@arg	goal 	Node object with x, y, theta
		"""
		return math.sqrt(math.pow((goal.x-self.x),2) + math.pow((goal.y-self.y),2))
