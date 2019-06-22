import os
import time
from Point2 import Point
import numpy as np
from mapTools import privacy_init, hasprivacythreat2, initialmapwithknowngrid, SaveMap
import copy
from Configure import configure
import math
import sys
from heapq import heappush
from pathinitial import PathInitial
from PathPlanningOnline import Astar_Path_Planning_online



grid_x = 10
grid_y = 10
grid_z = 10
safety_threshold = 0.3
privacy_threshold = 0.1
privacy_radius = [0.5, 1, 2]
# drone parameter
starting_point = Point(0, 0, 0, 0)
end_point = Point(9, 9, 9, 0)
viewradius = 2
Kca = 10

config = configure(grid_x, grid_y, grid_z, safety_threshold, privacy_threshold, privacy_radius, starting_point, end_point, viewradius)

SaveMap (config)

reinitial_flag = 1
refpath, len_refpath, sum_ref_initial, planpath, len_planpath, sum_plan_last = PathInitial(config, reinitial_flag)

while sum_ref_initial != sum_plan_last :
    reinitial_flag = 0
    refpath, len_refpath, sum_ref, planpath, len_planpath, sum_plan = PathInitial(config, reinitial_flag)
    sum_online_plan, len_trajectory_plan, num_intruder_plan, sum_pre, len_trajectory_ref, num_intruder_ref = Astar_Path_Planning_online (config)
    sum_ref_initial = sum_ref
