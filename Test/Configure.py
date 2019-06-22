#!/usr/bin/python
# -*- coding:utf-8 -*-
from Point2 import Point
#from Point import Point

class configure:
    def __init__(self):
        self.grid_x = 10
        self.grid_y = 10
        self.grid_z = 10
        self.grid = [self.grid_x, self.grid_y, self.grid_z]
        self.safety_threshold = 0.3
        self.privacy_threshold = 0.1
        # privacy_radius = 1 ##
        self.privacy_radius = [0.5, 1, 2]

        # drone parameter
        #self.starting_point1 = Point(0, 0, 0)
        #self.end_point1 = Point(4, 4, 4)

        self.starting_point = Point(0, 0, 0, 0)
        self.end_point = Point(9, 9, 9, 0)
        self.T_budget = (4/3)*(abs(self.end_point.x-self.starting_point.x) + abs(self.end_point.y-self.starting_point.y) + abs(self.end_point.z-self.starting_point.z))
        self.T_optimal = self.T_budget * 0.9
        self.viewradius = 2
        self.Kca = 10
