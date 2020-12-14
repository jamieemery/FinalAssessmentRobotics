#!/usr/bin/env python

"""
Find the optimal TSP solution using different algorithms
"""

import math
import random
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
	    length = self.calculate_length(newPathOrdered)
	    print("Length of greedy ="+str(length))
	    return newPathOrdered


	def calculate_length(self, path):
	    l = 0
	    i = 0
	    while i < len(path)-1:
		dist = path[i].euclidean_distance(path[i+1])
		l += dist
		i += 1
	    
	    return l


	def bruteTSPAlgorithm(self, start, listOfNodes):
	    possiblePaths = list(permutations(listOfNodes))
	    first_path = list(possiblePaths[0])
	    first_path.insert(0,start)
	    min_path_length = self.calculate_length(first_path)
	    smallest_path = first_path
	    for path in possiblePaths:
		path = list(path)
		path.insert(0,start)
		length = self.calculate_length(path)
		if length <= min_path_length:
		   min_path_length = length
		   smallest_path = path
	    print("Length of brute= "+str(min_path_length))
	    smallest_path.remove(start)
	    smallest_path = tuple(smallest_path)
	    return smallest_path
	

	def swapPositions(self, l, pos1, pos2): 
    	    l[pos1], l[pos2] = l[pos2], l[pos1] 
            return l

	
	def swapTSPAlgorithm(self, start, listOfNodes):
	    best_path = []
	    best_length = 1000000000000 #assign an irreasonably big number
	    # try 15 different initial paths
	    for i in range(15):
		path = listOfNodes
		random.shuffle(path)
		path.insert(0,start)
		length = self.calculate_length(path)
		for j in range(len(path)-1):
		    if j != 0:
		       new_path = self.swapPositions(path,j,j+1)
		       new_length = self.calculate_length(new_path)
		       if new_length < length:
			   length = new_length
			   path = new_path
		if length <= best_length:
		    best_length = length
		    best_path = path
		path.remove(start)
	    print("Length of swap ="+str(best_length))
	    return best_path