#!/usr/bin/env python

"""
Find the optimal path solution using the greedy TSP algorithm
"""

import math
from our_node import Node
from itertools import permutations

class TSP:


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


	def calculate_length(self, path):
	    l = 0
	    i = 0
	    while i < 5:
		dist = path[i].euclidean_distance(path[i+1])
		l += dist
		i += 1
	    
	    return l


	def bruteTSPAlgorithm(self, listOfNodes):
	    possiblePaths = list(permutations(listOfNodes))
	    min_path_length = self.calculate_length(possiblePaths[0])
	    smallest_path = possiblePaths[0]
	    for path in possiblePaths:
		length = self.calculate_length(path)
		if length <= min_path_length:
		   min_path_length = length
		   smallest_path = path

	    return smallest_path
	
            
	    
	    


	

