#!/usr/bin/env python

"""
Find the optimal path solution using the greedy TSP algorithm
"""

import math
from our_node import Node

class GreedyTSP:


	def takeSmallestDistanceNode(self, givenNode, nonTravelled):
	    prevEucDis = 0
	    for node in nonTravelled:
		nodeEucl = node.euclidean_distance(givenNode)
		if prevEucDis == 0 or prevEucDis > nodeEucl:
		    smallestNode = node
		prevEucDis = nodeEucl
	    nonTravelled.remove(smallestNode)
	    return smallestNode, nonTravelled	


	def greedyTSPAlgorithm(self, start, listOfNodes):
	    newPathOrdered = []
	    smallestNode, listOfNodes = self.takeSmallestDistanceNode(start, listOfNodes)
	    newPathOrdered.append(smallestNode)
	    while len(listOfNodes) > 0:
		smallestNode, listOfNodes = self.takeSmallestDistanceNode(smallestNode, listOfNodes)
		newPathOrdered.append(smallestNode)

	    return newPathOrdered


	

