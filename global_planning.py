#!/usr/bin/env python
# main function
import numpy as np
from path import Path
from point import Point
import geneticAlgorithm as gA
from quickSort import quick_sort
from gridVisualization import grid_visualization
from mapTools import privacy_init, map_generate
import ga
import copy
import math


grid_x = 10
grid_y = 10
grid_z = 10
safety_threshold = 0.5
privacy_threshold = 0.1
privacy_radius = 1


starting_point = Point(0, 0, 0, 0)
end_point = Point(4, 4, 4, 0)
T_budget = 20


# import global map

occ_grid, obstacle_num = map_generate(grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold)
pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, starting_point, end_point, occ_grid, privacy_radius)


# list of initial solution with global map
trajectory_ref = ga.initialsolution(occ_grid, pri_grid, starting_point, end_point, T_budget)
trajectory_plan = copy.deepcopy(trajectory_ref[:])
sensor_initial = np.zeros(T_budget)
sensor_plan = copy.deepcopy(sensor_initial[:])
time_step = 0

idx = 0


def hasprivacythreat (position, occ_grid, viewradius = 2):
    x = position.x
    y = position.y
    z = position.z
    r = viewradius
    flag = 0
    min_x = max(x - r, 0)
    max_x = min(x + r, grid_x - 1)
    min_y = max(y - r, 0)
    max_y = min(y + r, grid_y - 1)
    min_z = max(z - r, 0)
    max_z = min(z + r, grid_z - 1)
    for m in range(min_x, max_x + 1):
        for n in range(min_y, max_y + 1):
            for l in range(min_z, max_z + 1):
                if occ_grid[m][n][l] == 2 || occ_grid[m][n][l] == 3 ||occ_grid[m][n][l] == 4 :
                    dis = np.sqrt(np.power((x - m), 2) + np.power((y - n), 2) + np.power((z - l), 2))
                    if dis <= r:
                        flag = 1
    return flag



while not (idx >= len(trajectory_plan)):
    current_p = trajectory_plan[idx].point
    current_ca = trajectory_plan[idx].ca
    next_p = trajectory_plan[idx+1].point
    next_idx = idx + 1

    if current_p == end_point :
        break

    if current_ca == 1:
        time_step += 1
        current_p = next_p
        idx += 1
        continue

    if hasprivacythreat(current_p, occ_grid, 2) :
        # localization
        p_threat, h_impact = privacy_modeling()
        # update occ_grid, pri_grid

        for j in range (idx+1, len(trajectory_plan)):
            sigma_privacy = 0
            for k in range (j,len(trajectory_plan)):
                sigma_privacy += pri_grid * math.exp(-sensor_plan[j])
                if sigma_privacy == 0:
                    next_p = trajectory_plan[j].point
                    next_idx = j
                    break
                elif k == len(trajectory_plan)-1 :
                    next_p = trajectory_plan[-1].point
                    next_idx = len(trajectory_plan)-1
        T_plan = T_budget - idx - (len(trajectory_plan) - 1 - next_idx)
        trajectory_optimal, sensor_optimal = ga.motionplan(occ_grid, pri_grid, current_p, next_p, T_plan)
        previous_trajectroy = copy.deepcopy(trajectory_plan[ :idx])
        following_trajectroy = copy.deepcopy(trajectory_plan[next_idx+1: ])
        now_trajectroy = previous_trajectroy + trajectory_optimal + following_trajectroy
        trajectory_plan = copy.deepcopy(now_trajectroy)
    time_step += 1
    idx = next_idx





