import os
import time
from Point2 import Point
import numpy as np
from random import randint
from mapTools import privacy_init, hasprivacythreat2, initialmapwithknowngrid, SaveMap
import copy
from Configure import configure
import math
import sys
from heapq import heappush
from pathinitial import PathInitial
from PathPlanningOnline import Astar_Path_Planning_online
from HybridPlanningOnline import Astar_Hybrid_Planning_online
from SensorConfigOnline import Astar_Sensor_Config_online

from log import Log
log = Log(__name__, log_cate="results_0623-2" ).getlog()

for i in range (10):
    iteration = i
    grid_x = 10 + int(i/100)
    grid_y = 10 + int(i/100)
    grid_z = 10 + int(i/100)

    # safety_threshold, privacy_threshold 固定 0.3 最佳
    # safety_threshold_list = [0.2, 0.3, 0.4]
    # safety_threshold = safety_threshold_list[i%3]
    # privacy_threshold_list = [0.05, 0.1, 0.15]
    # privacy_threshold = privacy_threshold_list[i % 3]
    safety_threshold = 0.3
    privacy_threshold = 0.1
    privacy_radius = [0.5, 1, 2]

    # drone parameter
    # x1 = randint(0, grid_x - 1)
    # y1 = randint(0, grid_y - 1)
    # z1 = randint(0, grid_z - 1)
    # x2 = randint(0, grid_x - 1)
    # y2 = randint(0, grid_y - 1)
    # z2 = randint(0, grid_z - 1)
    # while x2==x1 and y2==y1 and z2==z1:
    #     x2 = randint(0, grid_x - 1)
    #     y2 = randint(0, grid_y - 1)
    #     z2 = randint(0, grid_z - 1)
    x1 = 0
    x2 = grid_x - 1
    y1 = 0
    y2 = grid_y - 1
    z1 = 0
    z2 = grid_z - 1
    starting_point = Point(x1, y1, z1, 0)
    end_point = Point(x2, y2, z2, 0)

    # alpha ,beta 固定
    # alpha_list = [4/2, 5/3, 6/4, 7/5, 8/6, 9/7, 10/8, 11/9, 12/10, 13/11]
    # alpha = alpha_list[i % 10]
    # beta_list = [3/2, 4/3, 5/4, 6/5, 7/6, 8/7, 9/8, 10/9, 11/10, 12/11]
    # beta = beta_list[i % 10]
    alpha = 5/3
    beta = 4/3
    viewradius = 2
    Kca = 10

    config = configure(grid_x, grid_y, grid_z, safety_threshold, privacy_threshold, privacy_radius, starting_point,
                       end_point, viewradius, alpha, beta)
    T_budget = alpha * (abs(x1-x2) + abs(y1-y2) + abs(z1-z2))
    T_optimal = beta * (abs(x1-x2) + abs(y1-y2) + abs(z1-z2))
    log.info("Iteration: %d; Configuration: grid: %d, safety_threshold: %f, privacy_threshold: %f, the starting point: [%d, %d, %d]; the end point: [%d, %d, %d]; T_budget(alpha): "
             "%f (%f); "
             "T_optimal(beta): %f (%f)"
             %(iteration, grid_x, safety_threshold, privacy_threshold, x1, y1, z1, x2, y2, z2, T_budget, alpha, T_optimal, beta) )
    SaveMap(config, iteration)

    reinitial_flag = 1
    refpath = []
    planpath = []
    refpath, len_refpath, sum_ref_initial, planpath, len_planpath, sum_plan_last, no_solution_flag = PathInitial(config, reinitial_flag,
                                                                                               iteration, log)
    if no_solution_flag != 1:
        log.info("Error for no initial solution!")
        continue
    num = 0
    while sum_ref_initial > sum_plan_last:
        num += 1
        refpath, len_refpath, sum_ref, planpath, len_planpath, sum_plan,  no_solution_flag = PathInitial(config, reinitial_flag, iteration,
                                                                                      log)
        sum_online_plan, len_trajectory_plan, num_intruder_plan, sum_pre, len_trajectory_ref, num_intruder_ref = Astar_Hybrid_Planning_online(
            config, iteration, log)
        reinitial_flag = 0
        sum_ref_initial = sum_ref
        if num > 30:
            log.info("Error to be trapped!")
            break

    reinitial_flag = 1
    refpath, len_refpath, sum_ref_initial, planpath, len_planpath, sum_plan_last,  no_solution_flag  = PathInitial(config, reinitial_flag,
                                                                                               iteration, log)
    num = 0
    while sum_ref_initial > sum_plan_last:
        num += 1
        reinitial_flag = 0
        refpath, len_refpath, sum_ref, planpath, len_planpath, sum_plan,  no_solution_flag  = PathInitial(config, reinitial_flag, iteration,
                                                                                      log)
        sum_online_plan, len_trajectory_plan, num_intruder_plan, sum_pre, len_trajectory_ref, num_intruder_ref = Astar_Sensor_Config_online(
            config, iteration, log)
        sum_ref_initial = sum_ref
        if num > 30:
            log.info("Error to be trapped!")
            break

    reinitial_flag = 1
    refpath, len_refpath, sum_ref_initial, planpath, len_planpath, sum_plan_last,  no_solution_flag  = PathInitial(config, reinitial_flag,
                                                                                               iteration, log)

    num = 0
    while sum_ref_initial > sum_plan_last:
        num += 1
        reinitial_flag = 0
        refpath, len_refpath, sum_ref, planpath, len_planpath, sum_plan,  no_solution_flag  = PathInitial(config, reinitial_flag, iteration,
                                                                                      log)
        sum_online_plan, len_trajectory_plan, num_intruder_plan, sum_pre, len_trajectory_ref, num_intruder_ref = Astar_Path_Planning_online(
            config, iteration, log)
        sum_ref_initial = sum_ref
        if num > 30:
            log.info("Error to be trapped!")
            break