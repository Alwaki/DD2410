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
from dubins import *
import heapq as h
import math as m
import time as t #TESTING; REMOVE LATER

#create instance of car for testing purposes
car = Car()

# Create Node class for storing data values
class Node():
    def __init__(self, x, y, theta, phi, costC, costG, cost, time, parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.costC = costC
        self.costG = costG
        self.cost = cost
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

# Description:
# In:
# Out:
def outsideBounds(car, xn, yn):
    if not car.xlb <= xn <= car.xub: # Check that x is inside left and right map bounds
      return True
    if not car.ylb <= yn <= car.yub: # Check that y is inside lower and upper map bounds
      return True
    return False

# Description:
# In:
# Out:
def collision(car, xn, yn):
    for i in range(len(car.obs)): # Check for every obstacle that the x and y coordinate does not result in collision
        if (car.obs[i][0] - car.obs[i][2] - 0.1 <= xn <= car.obs[i][0] + car.obs[i][2] + 0.1) and (car.obs[i][1] - car.obs[i][2] - 0.1 <= yn <= car.obs[i][1] + car.obs[i][2] + 0.1):
            return True
    return False

def goal(car, current):
    xt = car.xt
    yt = car.yt
    x = current.x
    y = current.y

    if (xt - 1.5 <= x <= xt + 1.5) and (yt - 1.5 <= y <= yt + 1.5): #goal tolerance of 1.5m
        return True
    return False

# Description:
# In:
# Out:
def Astar(car, current):
    minHeap = [] #initialize heap as list
    childHeap = [] #initialize secondary minheap
    visited = [] #intiialize visited list
    h.heappush(minHeap, current) #push initial node onto heap
    # Iterate through nodes in priority queue
    while not (goal(car, current) or minHeap == []):
        current = h.heappop(minHeap)
        print("x: " + str(current.x))
        print("y: " + str(current.y))
        for phi in [-m.pi/4, 0, m.pi/4]: #Full turns or straight are optimal, according to Pontryagins maximum principle
            #calculate new values for each phi (steering angle)
            xn, yn, thetan = step(car, current.x, current.y, current.theta, phi)

            #control feasibility of position
            if not outsideBounds(car, xn, yn):
                if not collision(car, xn, yn):
                    #calculate costs for these directives
                    costC = current.costC + m.hypot(car.x0-xn,car.y0-yn) #cost of travel from start position
                    costG = m.hypot(xn-car.xt,yn-car.yt) #current optimal distance to goal
                    cost = costC + costG
                    #print(Cost)
                    #renew time stamp
                    newTime = current.time + 0.01

                    #create child from new data
                    child = Node(xn, yn, thetan, phi, costC, costG, cost, newTime, current)
                    #push child onto heap
                    h.heappush(minHeap, child)
    return current

# Description:
# In:
# Out:
def solution(car):
    start = Node(car.x0, car.y0, 0, 0.2, 0, 0, 0, 0)
    end = Astar(car, start)
    print(end.x)
    print(end.y)

    return controls, times

# DRIVER CODE (for testing)
# ---------------------
if __name__ == '__main__':
    solution(car)
