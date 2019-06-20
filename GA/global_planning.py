#!/usr/bin/env python
# main function
import numpy as np
from GA.path import Path
from GA.point import Point
from GA import geneticAlgorithm as gA
from GA.mapTools import privacy_init, map_generate
from GA.ga_class import GA_class
import copy
import math
import sys
sys.setrecursionlimit(1000000)

#map parameter
grid_x = 50
grid_y = 50
grid_z = 50
safety_threshold = 0.3
privacy_threshold = 0.1
# privacy_radius = 1 ##
privacy_radius = [0.5,1,2]

# drone parameter
starting_point = Point(3, 3, 3, 0)
end_point = Point(7, 7, 7, 0)
T_budget = 100
viewradius = 2
Kca = 10

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
                    # print (occ_grid_known[i][j][k], i,j,k)
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    print(occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known,pri_grid,privacy_sum)
    return occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known

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


if __name__ == "__main__":
    occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap (grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold, privacy_radius)
    # print(occ_grid,obstacle_num,occ_grid_known,pri_grid_known,privacy_sum_known)

    # list of initial solution with global map
    """ to organize"""
    ga = GA_class(population, generation, selection_size)
    alg = gA.GeneticAlgorithm(population)

    max_flag = 1
    max_f = - 100
    trajectory_ref = None
    while max_flag == 1:
        max_f, trajectory_ref, max_flag = ga.initialsolution(occ_grid_known, pri_grid_known, privacy_sum_known, obstacle_num, starting_point, end_point, T_budget, 0, grid_x, grid_y, grid_z)
        print(max_f, max_flag)
        for j in range(len( trajectory_ref.points)):
            print(trajectory_ref.points[j])

    trajectory_plan = copy.deepcopy(trajectory_ref)
    sensor_initial = np.zeros(len(trajectory_plan.points))
    sensor_plan = copy.deepcopy(sensor_initial)
    time_step = 0

    idx = 0
    '''
    for j in range(len(trajectory_plan.points)):
        print(trajectory_plan.points[j])
    for j in range(len(sensor_plan)):
        print(sensor_plan[j])
    '''
    current_f = max_f
    while not (idx >= len(trajectory_plan.points)):
        current_p = trajectory_plan.points[idx]
        current_ca = trajectory_plan.points[idx].ca

        if current_p == end_point :
            break

        next_p = trajectory_plan.points[idx+1]
        next_idx = idx + 1
        print (current_p,current_ca,next_p,next_idx)

        if current_ca == 1:
            time_step += 1
            current_p = next_p
            idx += 1
            continue
        # take picture
        # update occ_grid, pri_grid
        flag, occ_grid_known, pri_grid_known, privacy_sum_known = hasprivacythreat (current_p, occ_grid_known, occ_grid, pri_grid_known, privacy_sum_known, viewradius)

        if flag:
            # localization
            # p_threat, h_impact = privacy_modeling()
            # update occ_grid, pri_grid
            # print(pri_grid_known, len(trajectory_plan.points))
            for j in range (idx+1, len(trajectory_plan.points)):
                sigma_privacy = 0
                for k in range (j,len(trajectory_plan.points)):
                    sigma_privacy += pri_grid_known[trajectory_plan.points[k].x][trajectory_plan.points[k].y][trajectory_plan.points[k].z] * math.exp(-(trajectory_plan.points[k].ca))
                if sigma_privacy == 0:
                    next_p = trajectory_plan.points[j]
                    next_idx = j
                    break
                elif k == len(trajectory_plan.points)-1 :
                    next_p = trajectory_plan.points[-1]
                    next_idx = len(trajectory_plan.points)-1
            print(next_idx,next_p)
            if next_idx != idx + 1: # no need for motion planning

                T_plan = T_budget - idx - (len(trajectory_plan.points) - 1 - next_idx)
                #if T_plan < (abs(trajectory_plan.points[next_idx].x - trajectory_plan.points[idx].x) + abs(trajectory_plan.points[next_idx].y - trajectory_plan.points[idx].y) + \
                #        abs(trajectory_plan.points[next_idx].z - trajectory_plan.points[idx].z)):
                #    print("no solution!")
                """ to organize"""
                print(T_plan, current_p,  next_p)
                trajectory_optimal_f, trajectory_optimal, trajectory_optimal_flag = ga.motionplan(occ_grid_known, pri_grid_known, privacy_sum_known, obstacle_num, current_p, next_p, T_plan, Kca, grid_x, grid_y, grid_z)
                previous_trajectory = copy.deepcopy(trajectory_plan.points[ :idx])
                following_trajectory = copy.deepcopy(trajectory_plan.points[next_idx+1: ])
                #for i in range (len(previous_trajectory.points))

                now_trajectory = []
                for ll in range(idx):
                    temp = Point(trajectory_plan.points[ll].x, trajectory_plan.points[ll].y,
                                 trajectory_plan.points[ll].z, trajectory_plan.points[ll].ca)
                    now_trajectory.append(temp)

                for ll in range(0, len(trajectory_optimal.points)):
                    temp = Point(trajectory_optimal.points[ll].x,trajectory_optimal.points[ll].y,trajectory_optimal.points[ll].z,trajectory_optimal.points[ll].ca)
                    now_trajectory.append(temp)

                #for ll in range(len(now_trajectory.points)):
                #    print(now_trajectory.points[ll])

                for ll in range(next_idx+1,len(trajectory_plan.points)):
                    temp = Point(trajectory_plan.points[ll].x,trajectory_plan.points[ll].y,trajectory_plan.points[ll].z,trajectory_plan.points[ll].ca)
                    now_trajectory.append(temp)

                #for ll in range(len(now_trajectory.points)):
                #    print(now_trajectory.points[ll])

                now_trajectory = Path(now_trajectory)
                for ll in range(len(now_trajectory.points)):
                    print(now_trajectory.points[ll])
                print("The length of now_trajectory: ", len(now_trajectory.points))
                #now_trajectory = previous_trajectory + trajectory_optimal + following_trajectory
                current_max_f = alg.get_fitness(trajectory_plan, occ_grid_known, pri_grid_known,
                                            starting_point, end_point, privacy_sum_known, obstacle_num)
                trajectory_plan = copy.deepcopy(now_trajectory)
                current_f = alg.get_fitness(trajectory_plan, occ_grid_known, pri_grid_known,
                                                   starting_point, end_point, privacy_sum_known, obstacle_num)
                print("fitness", current_max_f, current_f)
        time_step += 1
        idx = idx + 1
        print(idx,time_step, len(trajectory_plan.points))

