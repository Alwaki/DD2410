#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Alexander Wallen Kiessling}
# {19970408-0754}
# {akie@kth.se}

# Course DD2410
# Project: Assignment 3
# Info: This assignment concerns planning the path of a Dubin's car.
# The algorithm chosen for this problem is A*.


# SETUP
# ---------------------

# Import packages
from dubins import *
import heapq as h
import math as m
import time as t #TESTING; REMOVE LATER

#create instance of car for testing purposes
#car = Car()

# Create Node class for storing data values
class Node():
    def __init__(self, x, y, theta, phi, cost, travelled, time = 0, parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.cost = cost
        self.travelled = travelled
        self.time = time
        self.parent = parent

    # Object operators for comparison in heap
    def __eq__(self, other):
        return self.cost == other.cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __gt__(self, other):
        return self.cost > other.cost


# FUNCTION DEFINITIONS
# ---------------------

# Description: Controls that the proposed coordinate is within stated map bounds,
# returns True if the coordinates are invalid and False otherwise.
# In: Car class instance, x-coordinate (float) and y-coordinate (float)
# Out: Boolean truth statement
def outsideBounds(car, xn, yn):
    if not car.xlb <= xn <= car.xub: # Check that x is inside left and right map bounds
      return True
    if not car.ylb <= yn <= car.yub: # Check that y is inside lower and upper map bounds
      return True
    return False

# Description: Controls if the proposed coordinate results in a collision with
# the map obstacles. Simplifies obstacles as squares with sides 2*radius of circle.
# returns true for collision, and false otherwise.
# In: Car class instance, x-coordinate (float) and y-coordinate (float)
# Out: Boolean truth statement
def collision(car, xn, yn):
    for i in range(len(car.obs)): # Check for every obstacle that the x and y coordinate does not result in collision
        if (car.obs[i][0] - car.obs[i][2] - 0.1 <= xn <= car.obs[i][0] + car.obs[i][2] + 0.1) and (car.obs[i][1] - car.obs[i][2] - 0.1 <= yn <= car.obs[i][1] + car.obs[i][2] + 0.1):
            return True
    return False

# Description: Checks if the proposed coordinate has already been visited. Returns
# true if the coordinate has been visited, otherwise it adds the coordinate to the
# table, and returns false. As such, only one check can return false per coordinate.
# In: x-coordinate (float) and y-coordinate (float) as well as hashtable (python
# dictionary)
# Out: Boolean truth statement
def visited(xn, yn, table):
    if xn in table:
        if table[xn] == yn:
            return True
    table[xn] = yn
    return False

# Description: Gathers other boolean functions to check if coordinates are valid.
# In: see used functions descriptions.
# Out: Boolean truth statement.
def validCheck(car, xn, yn, table):
    return not (collision(car, xn, yn) and outsideBounds(car, xn, yn) and visited(xn, yn, table))

# Description: Checks if coordinates are within specified goal area, returns
# true if within goal area, false otherwise.
# In: Car class instance, node data type which contains coordinate.
# Out: Boolean truth statement.
def goal(car, current):
    xt = car.xt
    yt = car.yt
    x = current.x
    y = current.y

    if (m.hypot(current.x-car.xt,current.y-car.yt)) <= 1.5: #goal tolerance of 1.5m
        return True
    return False

# Description: Pathfinding algorithm, iteratively generates new neighbouring
# nodes and selects the cheapest of these through utilizing a min heap. The cost
# function is f(x) = c(x) + g(x), where c(x) is the cumulative sum of cost of
# travelling from x-1 to x, and g(x) is the cost of the optimal path (ignoring
# obstacles) from x to the goal.
# In: Car class object, a node as starting point.
# Out: The finishing node, with attached parent pointers.
def Astar(car, current):
    minHeap = [] #initialize heap as list
    hashTable = {} #initialize hash table for checking visited nodes
    h.heappush(minHeap, current) #push initial node onto heap
    heapCount = 1 #add upon pushes to heap, subtract upon pop
    # Iterate through nodes in priority queue
    while not ((goal(car, current)) or heapCount == 0):
        current = h.heappop(minHeap)
        heapCount -= 1
        options = [] #list for picking the best of children
        for phi in [-m.pi/4, 0, m.pi/4]: #Full turns or straight are optimal, according to Pontryagins maximum principle
            #calculate new values for each phi (steering angle)
            xn, yn, thetan = step(car, current.x, current.y, current.theta, phi)
            #control feasibility of position
            if validCheck(car, xn, yn, hashTable):
                #calculate costs for these directives
                costC = current.travelled + m.hypot(current.x - xn, current.y - yn) #cost of travel from start position
                costG = m.hypot(car.xt - xn, car.yt - yn) #current optimal distance to goal
                totalCost = costC + costG
                #renew time stamp
                newTime = current.time + 0.01
                #create child from new data
                child = Node(xn, yn, thetan, phi, totalCost, costC, newTime, current)
                #push child onto heap
                options.append(child)
        if options == []:
            h.heappush(minHeap, current.parent) #if none of children are viable, step back one node
        else:
            h.heappush(minHeap, min(options))
    return current

# Description: Recreates the path taken by a pathfinding algorithm.
# In: A node with attached parent pointers
# Out: Arrays of data from the nodes travelled in the order of the path taken.
def pathwrite(endNode):
    current = endNode
    controls = [] #initialize array
    times = [] #initialize array
    while current != None: #trace path from end to start, append to array
        controls.append(current.phi)
        times.append(current.time)
        current = current.parent #change to new node

    controls.reverse() #reverse path arrays, for correct order instruction set
    times.reverse()
    times.append(endNode.time + 0.1) #add 1 extra time stamp according to instructions
    return controls, times

# Description: Creates a solution for moving a car from an initial point to
# a point within a distance of a specified coordinate, whilst staying within
# bounds, avoiding obstacles and remaining efficient.
# In: Car class object.
# Out: Arrays with a steering angle for the car, as well as time stamps with
# increments of 0.01 seconds.
def solution(car):
    startNode = Node(car.x0, car.y0, 0, 0.2, 0, 0, 0)
    endNode = Astar(car, startNode)
    controls, times = pathwrite(endNode)
    return controls, times

# DRIVER CODE (for testing)
# ---------------------
if __name__ == '__main__':
    solution(car)
