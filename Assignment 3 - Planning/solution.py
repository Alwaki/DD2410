#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Alexander Wallen Kiessling}
# {19970408-0754}
# {akie@kth.se}

# Course DD2410
# Project: Assignment 3
# Info: This assignment concerns planning the path of a Dubin's car.
# The algorithm chosen for this problem is RRT.
# Additionally, the algorithm is based on cpslab.snu.ac.kr
# instructions for assignment 2.

# SETUP
# ---------------------

# Import packages
from dubins import *
import numpy as np

# Create instance
car = Car()

# DEFINITIONS
# ---------------------
class tree(object):
    def __init__(self, car):
        self.car = car
        self.vertices = []
        self.iteration_limit = 3000
        self.bias = 4
        self.backward_limit = 15
        self.forward_limit = 200

    def build_tree(self):
        goal_found = False
        print("The thought goal is in x = " + str(car.xt) + " y = " + str(car.yt))
        for iteration in range(self.iteration_limit):
            random = self.random_state(iteration)
            nearest = self.nearest_neighbour(random)
            new = vertex()
            new.set_position(nearest.x_current, nearest.y_current, nearest.theta_current)
            new.parent = nearest
            for i in range(self.forward_limit):
                phi = self.select_input(new, random)
                xn, yn, thetan = step(car, new.x_current, new.y_current, new.theta_current, phi)

                if self.car._environment.safe(xn, yn):
                    new.phi.append(phi)
                    new.x.append(xn)
                    new.y.append(yn)
                    new.theta.append(thetan)
                    new.set_position(xn, yn, thetan)
                    if ((car.xt-xn)**2 + (car.yt-yn)**2)**0.5 < 1.0:
                        goal_found = True
                        break
                else:
                    try:
                        for i in range(self.backward_limit):
                            del new.x[-1]
                            del new.y[-1]
                            del new.theta[-1]
                            del new.phi[-1]
                        new.set_position(new.x[-1],new.y[-1], new.theta[-1])
                    except:
                        pass
            self.vertices.append(new)
            if goal_found == True:
                print("feasible path found")
                return new
        print("no feasible path")
        return new

    def random_state(self, iteration):
        if iteration % self.bias == 0:
            return self.car.xt, self.car.yt
        return np.random.uniform(self.car.xlb, self.car.xub), np.random.uniform(self.car.ylb, self.car.yub)

    def nearest_neighbour(self, random):
        distance = 2048
        nearest = None
        for item in self.vertices:
            #avoid complex calculations to save computational power
            temp_distance = np.hypot((item.x_current - random[0]), (item.y_current - random[1]))
            if temp_distance < distance:
                distance = temp_distance
                nearest = item
        return nearest

    def select_input(self, nearest, random):
        desired_theta = np.arctan2(random[0] - nearest.x_current, random[1] - nearest.y_current) - nearest.theta_current
        if desired_theta > 0.005:
            return np.pi/4
        if desired_theta < -0.005:
            return -np.pi/4
        return 0

    def extract_path(self, vertex):
        control_list = []

        while vertex.parent != None:
            control_list.append(vertex.phi)
            vertex = vertex.parent

        control_list.reverse()
        controls = []
        for i in control_list:
            for j in i:
                controls.append(j)
        times = [0]
        for i in range(len(controls)):
            times.append(i * 0.01 + 0.01)
        return controls, times


class vertex():
    def __init__(self):
        self.x = []
        self.y = []
        self.theta = []
        self.phi = []
        self.parent = None
        self.x_current = None
        self.y_current = None
        self.theta_current = None

    def set_position(self, x, y, theta):
        self.x_current = x
        self.y_current = y
        self.theta_current = theta

def solution(car):
    #create vertex for start
    start = vertex()
    start.set_position(car.x0, car.y0, 0)

    #initialize tree with start vertex
    path_finder = tree(car)
    path_finder.vertices.append(start)

    #build tree to find feasible path
    end = path_finder.build_tree()

    #TESTING PRINTS
    print("The final node is in x = " + str(end.x_current) + " y = " + str(end.y_current))
    print("The actual goal is in x = " + str(car.xt) + " y = " + str(car.yt))

    #extract path from last vertex
    controls, times = path_finder.extract_path(end)

    return controls, times

# DRIVER CODE (test purposes)
# ---------------------
if __name__ == '__main__':
    pass
