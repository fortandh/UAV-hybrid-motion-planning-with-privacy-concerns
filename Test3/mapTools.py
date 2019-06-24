#!/usr/bin/python
# -*- coding:utf-8 -*-
# map generation tools

import numpy as np
from random import randint
import math
import copy
from Point2 import Point
from Configure import configure



# 隐私部分的初始化
def privacy_init(grid_x, grid_y, grid_z, occ_grid, radius):
    #print(radius[0])
    pri_grid = np.zeros((grid_x, grid_y, grid_z))
    for i in range(grid_x):
        for j in range(grid_y):
            for k in range(grid_z):
                # 这里隐私等级分为三级，数字越大，级别越高
                if (occ_grid[i][j][k] == 2) or (occ_grid[i][j][k] == 3) or (occ_grid[i][j][k] == 4):
                    # different level of privacy restricted area has different affecting radius:
                    temp = int (occ_grid[i][j][k])
                    r = radius[temp-2]
                    min_x = max(i - r, 0)
                    min_x = math.floor(min_x)
                    max_x = min(i + r, grid_x - 1)
                    max_x = math.ceil(max_x)
                    min_y = max(j - r, 0)
                    min_y = math.floor(min_y)
                    max_y = min(j + r, grid_y - 1)
                    max_y = math.ceil(max_y)
                    min_z = max(k - r, 0)
                    min_z = math.floor(min_z)
                    max_z = min(k + r, grid_z - 1)
                    max_z = math.ceil(max_z)
                    for m in range(min_x, max_x + 1):
                        for n in range(min_y, max_y + 1):
                            for l in range(min_z, max_z + 1):
                                dis = np.sqrt(np.power((i - m), 2) + np.power((j - n), 2) + np.power((k - l), 2))
                                h = 0
                                if dis <= r:
                                    if occ_grid[i][j][k] == 2:
                                        h = 0.1
                                    elif occ_grid[i][j][k] == 3:
                                        h = 10
                                    elif occ_grid[i][j][k] == 4:
                                        h = 100
                                    # print (dis, np.power(dis, 2),math.exp((-1/2)*np.power(dis, 2)),i,j,k,m,n,l)
                                    # print (pri_grid[m][n][l])
                                    pri_grid[m][n][l] += h * math.exp((-1 / 2) * np.power(dis, 2))
                                    # print(pri_grid[m][n][l])
    sum_privacy = 0
    for i in range(grid_x):
        for j in range(grid_y):
            for k in range(grid_z):
                sum_privacy += pri_grid[i][j][k]
    return pri_grid, sum_privacy


def map_generate(grid_x, grid_y, grid_z, start, end, safety_threshold, privacy_threshold):
    map_volume = grid_x * grid_y * grid_z
    obstacle_num = map_volume * safety_threshold
    restricted_area_num = map_volume * privacy_threshold
    occ_grid = np.zeros((grid_x, grid_y, grid_z))
    occ_grid[start.x][start.y][start.z] = 7
    occ_grid[end.x][end.y][end.z] = 8
    i = 0
    # obstacle
    while i < obstacle_num:
        x = randint(0, grid_x - 1)
        y = randint(0, grid_y - 1)
        z = randint(0, grid_z - 1)
        if occ_grid[x][y][z] == 0:
            i = i+1
            occ_grid[x][y][z] = 1
    # private house
    i = 0
    while i < restricted_area_num:
        x = randint(0, grid_x - 1)
        y = randint(0, grid_y - 1)
        z = randint(0, grid_z - 1)
        if occ_grid[x][y][z] == 0:
            i = i + 1
            occ_grid[x][y][z] = randint(2, 4)
    return occ_grid, obstacle_num


# import global map
def initialmap (grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold, privacy_radius):
    #print("start")
    occ_grid, obstacle_num = map_generate(grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold)
    """ for testing
    occ_grid = np.array([[[7, 1, 1, 0, 0.],
 [3, 0, 0, 4, 0.],
 [4, 0, 1, 0, 1.],
 [1, 0, 0, 3, 1.],
 [0, 0, 1, 0, 1.]],
[[1, 1, 0, 0, 0.],
 [4, 0, 1, 1, 0.],
 [0, 2, 0, 1, 0.],
 [1, 0, 0, 1, 1.],
 [0, 3, 0, 0, 0.]],

[[0, 1, 1, 1, 0.],
 [0, 0, 1, 0, 0.],
 [0, 0, 0, 0, 1.],
 [1, 0, 0, 4, 1.],
 [0, 1, 0, 1, 0.]],

[[1, 0, 4, 1, 0.],
 [0, 0, 0, 0, 1.],
 [0, 0, 0, 0, 0.],
 [0, 0, 4, 2, 1.],
 [1, 1, 1, 0, 0.]],

[[0, 4, 0, 0, 0.],
 [1, 1, 0, 0, 0.],
 [0, 1, 0, 1, 1.],
 [1, 0, 0, 0, 3.],
 [0, 0, 0, 0, 8.]],
],

)
    obstacle_num = 0
    #"""
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    occ_grid_known = copy.deepcopy(occ_grid)

    for i in range (grid_x):
        for j in range (grid_y):
            for k in range (grid_z):
                if occ_grid[i][j][k] == 2 or occ_grid[i][j][k] == 3 or occ_grid[i][j][k] == 4:
                    occ_grid_known[i][j][k] = 0
                    # print (occ_grid_known[i][j][k], i,j,k)
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    # print(occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known,pri_grid,privacy_sum)
    return occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known

def initialmapwithknowngrid (grid_x, grid_y, grid_z, privacy_threshold, privacy_radius, occ_grid):

    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    occ_grid_known = copy.deepcopy(occ_grid)

    for i in range (grid_x):
        for j in range (grid_y):
            for k in range (grid_z):
                if occ_grid[i][j][k] == 2 or occ_grid[i][j][k] == 3 or occ_grid[i][j][k] == 4:
                    occ_grid_known[i][j][k] = 0
                    # print (occ_grid_known[i][j][k], i,j,k)
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    # print(occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known,pri_grid,privacy_sum)
    return occ_grid_known, pri_grid_known, privacy_sum_known

# 0624
def initialmapwithknowngrid_ratio (grid_x, grid_y, grid_z, privacy_threshold, privacy_radius, occ_grid, exploraton_rate):

    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    occ_grid_known = copy.deepcopy(occ_grid)

    map_volume = grid_x * grid_y * grid_z
    restricted_area_num = map_volume * privacy_threshold

    occ_grid_know_ratio = restricted_area_num * exploraton_rate

    i = 0

    for i in range(grid_x):
        for j in range(grid_y):
            for k in range(grid_z):
                if occ_grid[i][j][k] == 2 or occ_grid[i][j][k] == 3 or occ_grid[i][j][k] == 4:
                    occ_grid_known[i][j][k] = 0
                    # print (occ_grid_known[i][j][k], i,j,k)
    i = 0
    # 更新已探索的
    while i < occ_grid_know_ratio:
        x = randint(0, grid_x - 1)
        y = randint(0, grid_y - 1)
        z = randint(0, grid_z - 1)
        if (occ_grid[x][y][z] == 2 or occ_grid[x][y][z] == 3 or occ_grid[x][y][z] == 4) and occ_grid_known[x][y][z] == 0:
            i = i + 1
            occ_grid_known[x][y][z] = occ_grid[x][y][z]

    a= 0
    for i in range(grid_x):
        for j in range(grid_y):
            for k in range(grid_z):
                if occ_grid_known[i][j][k] != occ_grid[i][j][k]:
                    a += 1
    a = 1 - a / (grid_x * grid_y * grid_z * privacy_threshold)
    print("\033[94m exploration rate: \033[0m", a)


    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    # print(occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known,pri_grid,privacy_sum)
    return occ_grid_known, pri_grid_known, privacy_sum_known



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
                        # if occ_grid_known[m][n][l] != occ_grid[m][n][l]:
                        #     flag = 1
                        occ_grid_known[m][n][l] = occ_grid[m][n][l]
                        threat_list.append([m, n, l])
                        # update global risk model
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    return flag, occ_grid_known, pri_grid_known, privacy_sum_known

def hasprivacythreat2 (position, occ_grid_known, occ_grid, pri_grid_known, privacy_sum_known, config):
    x = position.x
    y = position.y
    z = position.z
    r = config.viewradius
    grid_x = config.grid_x
    grid_y = config.grid_y
    grid_z = config.grid_z
    privacy_radius = config.privacy_radius
    flag = 0
    min_x = max(x - r, 0)
    max_x = min(x + r, grid_x - 1)
    min_y = max(y - r, 0)
    max_y = min(y + r, grid_y - 1)
    min_z = max(z - r, 0)
    max_z = min(z + r, grid_z - 1)
    threat_list = []
    for m in range(min_x, max_x + 1):
        for n in range(min_y, max_y + 1):
            for l in range(min_z, max_z + 1):
                if occ_grid[m][n][l] == 2 or occ_grid[m][n][l] == 3 or occ_grid[m][n][l] == 4 :
                    dis = np.sqrt(np.power((x - m), 2) + np.power((y - n), 2) + np.power((z - l), 2))
                    ## different level of privacy threat has different radius to affect
                    #if dis <= r[occ_grid[m][n][l]-2]:
                    if dis <= r:
                        flag = 1
                        # if occ_grid_known[m][n][l] != occ_grid[m][n][l]:
                        #     flag = 1
                        occ_grid_known[m][n][l] = occ_grid[m][n][l]
                        threat_list.append([m, n, l])
                        # update global risk model
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    return flag, occ_grid_known, pri_grid_known, privacy_sum_known, threat_list

def map_of_city (grid_x, grid_y, grid_z, start, end, safety_threshold, privacy_threshold):
    occ_grid = np.zeros((grid_x, grid_y, grid_z))
    map_volume = grid_x *  grid_z
    building_num = map_volume * safety_threshold
    #building_num = 3
    restricted_area_num = map_volume * privacy_threshold
    occ_grid[start.x][start.y][start.z] = 7
    occ_grid[end.x][end.y][end.z] = 8
    i = 0
    # buildings
    buildings_side = [1,2,3,4]
    buildings_level = [1,2,3,4]
    num_obstacle = 0
    while i < building_num:
        flag = 0
        x = randint(0, grid_x - 1)
        z = randint(0, grid_z - 1)
        buildingside1 = buildings_side[randint(0, len(buildings_side)-1)]
        buildingside2 = buildings_side[randint(0, len(buildings_side) - 1)]
        buildinglevel = buildings_level[randint(0, len(buildings_level)-1)]

        # find free space
        if x + buildingside1 > grid_x or z + buildingside2 > grid_z :
            flag = 1
            continue
        for j in range (x, x + buildingside1):
            for k in range (z, z + buildingside2):
                if occ_grid[j][0][k] != 0:
                    flag = 1
                    break
        # house setting

        if flag == 0:
            i = i + 1
            print(x, z, buildingside1, buildingside2, buildinglevel)
            for j in range(x, x + buildingside1):
                for k in range(z, z + buildingside2):
                    for ll in range(0, buildinglevel):
                        occ_grid[j][ll][k] = 1
                        num_obstacle +=1

    i = 0
    while i < restricted_area_num:
        x = randint(0, grid_x - 1)
        #y = randint(0, grid_y - 1)
        z = randint(0, grid_z - 1)
        if occ_grid[x][0][z] == 0:
            i = i + 1
            occ_grid[x][0][z] = randint(2, 4)

    return occ_grid, num_obstacle


#occ_grid, num_obstacle = map_of_city (10, 10, 10, Point(0, 0, 0), Point(9, 0, 9), 0.1
#                                      , 0.05)
#print (occ_grid, num_obstacle)

def SaveMap (config, iteration, exploration_rate):
    grid_x = config.grid_x
    grid_y = config.grid_y
    grid_z = config.grid_z
    grid = config.grid
    safety_threshold = config.safety_threshold
    privacy_threshold = config.privacy_threshold
    # privacy_radius = 1 ##
    privacy_radius = config.privacy_radius

    # drone parameter
    starting_point = config.starting_point
    end_point = config.end_point
    T_budget = config.T_budget
    viewradius = config.viewradius
    Kca = config.Kca

    if iteration == 0:
        occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap(grid_x, grid_y, grid_z,
                                                                                               starting_point,
                                                                                               end_point,
                                                                                               safety_threshold,
                                                                                               privacy_threshold,
                                                                                               privacy_radius)
        occ_grid_known_initial, pri_grid_known, privacy_sum_known = initialmapwithknowngrid_ratio(grid_x, grid_y,
                                                                                                  grid_z,
                                                                                                  privacy_threshold,
                                                                                                  privacy_radius,
                                                                                                  occ_grid,
                                                                                                  exploration_rate)
        occ_grid_name = "occ_grid" + ".npy"
        np.save(file=occ_grid_name, arr=occ_grid)

    else:
        occ_grid_name = "occ_grid" + ".npy"
        occ_grid = np.load(file=occ_grid_name)

        occ_grid_known_initial, pri_grid_known, privacy_sum_known = initialmapwithknowngrid_ratio(grid_x, grid_y,
                                                                                                  grid_z,
                                                                                                  privacy_threshold,
                                                                                                  privacy_radius,
                                                                                                  occ_grid,
                                                                                                  exploration_rate)



    print(iteration)
    occ_grid_known_name = "occ_grid_known_initial" + str(iteration) + ".npy"
    np.save(file=occ_grid_known_name, arr=occ_grid_known_initial)
    # np.save(file="occ_grid_known.npy", arr=occ_grid_known)
    c = np.load(file="occ_grid_known_initial" + str(iteration) + ".npy")
    for m in range(grid_x):
        print("The value of x: ", m)
        print(c[m])


if __name__ == '__main__':
    config = configure()

    grid_x = config.grid_x
    grid_y = config.grid_y
    grid_z = config.grid_z
    grid = config.grid
    safety_threshold = config.safety_threshold
    privacy_threshold = config.privacy_threshold
    # privacy_radius = 1 ##
    privacy_radius = config.privacy_radius

    # drone parameter
    starting_point = config.starting_point
    end_point = config.end_point
    T_budget = config.T_budget
    viewradius = config.viewradius
    Kca = config.Kca


    occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap(grid_x, grid_y, grid_z,
                                                                                           starting_point, end_point,
                                                                                           safety_threshold,
                                                                                           privacy_threshold,
                                                                                           privacy_radius)
    np.save(file="occ_grid.npy", arr=occ_grid)
    b = np.load(file="occ_grid.npy")
    for m in range(grid_x):
        print("The value of x: ", m)
        print(b[m])
    np.save(file="occ_grid_known.npy", arr=occ_grid_known)
    c = np.load(file="occ_grid_known.npy")
    for m in range(grid_x):
        print("The value of x: ", m)
        print(c[m])