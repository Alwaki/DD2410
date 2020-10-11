#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Alexander Wallen Kiessling}
# {19970408-0754}
# {akie@kth.se}

# Course DD2410
# Task: Assignment 3
# Info: This assignment concerns planning the path of a Dubin's car.
# The algorithm chosen for this problem is RRT.

# SETUP
# ---------------------

# Import packages
from dubins import *
import numpy as np

# DEFINITIONS
# ---------------------

class Tree(object):
    '''
    Tree structure, vertices are a custom object.
    Iterates to find a path between a starting
    point and a goal point, adhering to certain
    constraints.
    '''
    def __init__(self, car):
        # Constructor
        self.car = car
        self.vertices = [Vertex(self.car.x0,self.car.y0)] #starting point
        self.forward_limit = 20
        self.goal_tolerance = 1
        self.bias = 15

    def build_tree(self):
        while True:
            # Get random point on map (with goal bias, returning goal sometimes)
            random = self.random_state()

            # Find nearest point from tree using indexing
            nearest_index = self.get_nearest(random)
            nearest = self.vertices[nearest_index]

            # Calculate appropriate driving angle
            phi = self.select_input(nearest, random)

            #Initialize data structures and flags
            collision = False
            forward_drive = 0
            x_array = []
            y_array = []
            theta_array = []

            #set current coordinate and heading values
            x_current = nearest.x
            y_current = nearest.y
            theta_current = nearest.theta

            #drive with given steering
            while (not collision) and (forward_drive < self.forward_limit):

                #step a distance correlating to dt time in driving direction
                x_current, y_current, theta_current = step(self.car, x_current, y_current, theta_current, phi)

                #append to arrays if successful drive
                x_array.append(x_current)
                y_array.append(y_current)
                theta_array.append(theta_current)

                #increase counter
                forward_drive = forward_drive + 1

                #check collision status with boundary or obstacle
                if not self.valid_check(x_current, y_current):
                    collision = True
                    forward_drive = forward_drive-1

            #add drive path to tree (if path exists)
            if forward_drive != 0:
                new = Vertex(x_array[forward_drive-1], y_array[forward_drive-1], theta_array[forward_drive-1], phi, forward_drive*0.01 + nearest.dt, nearest_index)
                self.vertices.append(new)

                #check if new vertex point is in goal zone
                distance_x = new.x - self.car.xt
                distance_y = new.y - self.car.yt
                distance = (distance_x**2 + distance_y**2)
                if distance < self.goal_tolerance:
                    break

        #Initialize arrays
        controls = []
        times = []
        last = len(self.vertices) - 1

        #Trace path along vertices, with timestamps
        while self.vertices[last].parent != None:
            vertex = self.vertices[last]
            controls.insert(0,vertex.phi)
            times.insert(0,vertex.dt)
            last = vertex.parent

        times.insert(0,0)
        return controls, times

    def get_nearest(self, random):
        distance_list = [(vertex.x - random[0]) ** 2 + (vertex.y - random[1])** 2 for vertex in self.vertices]
        nearest_vertex = distance_list.index(min(distance_list))
        return nearest_vertex

    def random_state(self):
        if np.random.randint(0, 100) > self.bias:
            return [np.random.uniform(self.car.xlb, self.car.xub), np.random.uniform(self.car.ylb, self.car.yub)]
        return [self.car.xt, self.car.yt]

    def select_input(self, nearest, random):

        #Calculate input angle phi using random and nearest vectors
        n_vector = np.array([np.cos(nearest.theta), np.sin(nearest.theta), 0])
        r_vector = np.array([random[0] - nearest.x, random[1] - nearest.y, 0])
        v_product = np.cross(n_vector, r_vector)
        cos_phi = n_vector.dot(r_vector)/(np.linalg.norm(n_vector)*np.linalg.norm(r_vector))
        steering_angle = np.arccos(cos_phi)

        #Check that phi does not exceed limits
        if v_product[2] < 0:
            steering_angle = steering_angle*-1
            if steering_angle < -1*np.pi/4:
                steering_angle = -1*np.pi/4
        if steering_angle > np.pi/4:
            steering_angle = np.pi/4
        return steering_angle

    def valid_check(self, x, y):

        #Check that coordinates are not inside obstacles
        for (obstacle_x, obstacle_y, obstacle_r) in self.car.obs:
            distance_x = obstacle_x - x
            distance_y = obstacle_y - y
            distance = (distance_x**2 + distance_y**2)
            if distance <= obstacle_r**2 + 0.1:
                return False

        #Check that coordinates are not outside bounds
        if not (self.car.xlb <= x <= self.car.xub - 0.1) or not (self.car.ylb + 0.1 <= y <= self.car.yub - 0.1):
            return False
        return True

class Vertex():
    '''
    Data storage object, stores the
    details of each visited position
    for tree, such that a path can
    be constructed.
    '''
    def __init__(self, x, y, theta = 0, phi = 0, dt = 0, parent = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.dt = dt
        self.parent = parent


def solution(car):
    '''
    Solution function, called by solver method
    for path integration
    '''
    #initialize tree with start vertex
    path_planning = Tree(car)

    #build tree to find feasible path
    controls, times = path_planning.build_tree()

    return controls, times
