#!/usr/bin/env python
# main function
import numpy as np
from path import Path
from point import Point
import geneticAlgorithm as gA
from quickSort import quick_sort
from gridVisualization import grid_visualization
from mapTools import privacy_init, map_generate
from ga_class import GA_class
import copy
import math
import sys
sys.setrecursionlimit(1000000)

#map parameter
grid_x = 5
grid_y = 5
grid_z = 5
safety_threshold = 0.3
privacy_threshold = 0.1
privacy_radius = 1 ## [1,2,3]

# drone parameter
starting_point = Point(0, 0, 0, 0)
end_point = Point(4, 4, 4, 0)
T_budget = 100
viewradius = 2
Kca = 0

# GA parameter
population = 500
generation = 5
selection_size = 50


# import global map
def initialmap (grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold, privacy_radius):
    #print("start")
    occ_grid, obstacle_num = map_generate(grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold)
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    occ_grid_known = copy.deepcopy(occ_grid)

    for i in range (grid_x):
        for j in range (grid_y):
            for k in range (grid_z):
                if occ_grid[i][j][k] == 2 or occ_grid[i][j][k] == 3 or occ_grid[i][j][k] == 4:
                    occ_grid_known[i][j][k] = 0
                    #print (occ_grid_known[i][j][k], i,j,k)
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    print (occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known)
    return occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known

occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap (grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold, privacy_radius)

# list of initial solution with global map
""" to organize"""
ga = GA_class(population, generation, selection_size)

max_flag =1
while max_flag == 1:
    max_f, trajectory_ref, max_flag = ga.initialsolution(occ_grid, pri_grid_known, privacy_sum_known, obstacle_num, starting_point, end_point, T_budget, 0)
    print(max_f,max_flag)
    for j in range(len( trajectory_ref.points)):
        print( trajectory_ref.points[j])


trajectory_plan = copy.deepcopy(trajectory_ref)
sensor_initial = np.zeros(len(trajectory_plan.points))
sensor_plan = copy.deepcopy(sensor_initial)
time_step = 0

idx = 0

for j in range(len(trajectory_plan.points)):
    print(trajectory_plan.points[j])
for j in range(len(sensor_plan)):
    print(sensor_plan[j])

def hasprivacythreat (position, occ_grid_known, occ_grid, pri_grid_known, privacy_sum_known, viewradius = 2):
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
                if occ_grid[m][n][l] == 2 or occ_grid[m][n][l] == 3 or occ_grid[m][n][l] == 4 :
                    dis = np.sqrt(np.power((x - m), 2) + np.power((y - n), 2) + np.power((z - l), 2))
                    ## different level of privacy threat has different radius to affect
                    #if dis <= r[occ_grid[m][n][l]-2]:
                    if dis <= r:
                        flag = 1
                        occ_grid_known[m][n][l] = occ_grid[m][n][l]
    # update global risk model
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    return flag, occ_grid_known, pri_grid_known, privacy_sum_known



while not (idx >= len(trajectory_plan.points)):
    current_p = trajectory_plan.points[idx]
    current_ca = trajectory_plan.points[idx].ca
    next_p = trajectory_plan.points[idx+1]
    next_idx = idx + 1
    print (current_p,current_ca,next_p,next_idx)
    if current_p == end_point :
        break

    if current_ca == 1:
        time_step += 1
        current_p = next_p
        idx += 1
        continue
    # take picture
    # update occ_grid, pri_grid
    flag, occ_grid_known, pri_grid_known, privacy_sum_known = hasprivacythreat (current_p, occ_grid_known, occ_grid, pri_grid_known, privacy_sum_known, viewradius)


    if flag :
        # localization
        #p_threat, h_impact = privacy_modeling()
        # update occ_grid, pri_grid
        print(pri_grid_known, len(trajectory_plan.points))
        for j in range (idx+1, len(trajectory_plan.points)):
            sigma_privacy = 0
            for k in range (j,len(trajectory_plan.points)):
                sigma_privacy += pri_grid_known[trajectory_plan.points[k].x][trajectory_plan.points[k].y][trajectory_plan.points[k].z] * math.exp(-sensor_plan[k])
            if sigma_privacy == 0:
                next_p = trajectory_plan.points[j]
                next_idx = j
                break
            elif k == len(trajectory_plan.points)-1 :
                next_p = trajectory_plan.points[-1]
                next_idx = len(trajectory_plan.points)-1
        T_plan = T_budget - idx - (len(trajectory_plan.points) - 1 - next_idx)
        """ to organize"""
        print(T_plan,current_p,  next_p)
        trajectory_optimal_f, trajectory_optimal, trajectory_optimal_flag = ga.motionplan(occ_grid_known, pri_grid_known, privacy_sum_known, obstacle_num, current_p, next_p, T_plan, Kca)
        previous_trajectroy = copy.deepcopy(trajectory_plan.points[ :idx])
        following_trajectroy = copy.deepcopy(trajectory_plan.points[next_idx+1: ])
        now_trajectroy = []
        now_trajectroy.append(previous_trajectroy)
        now_trajectroy.append(trajectory_optimal)
        now_trajectroy.append(following_trajectroy)
        #now_trajectory = previous_trajectroy + trajectory_optimal + following_trajectroy
        print(now_trajectroy)
        trajectory_plan = copy.deepcopy(now_trajectroy)
    time_step += 1
    idx = next_idx





