# map generation tools

import numpy as np
from random import randint
import math


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


'''
def map_init(grid_x, grid_y, grid_z, thickness, obstacles, goals):
    occ_grid = np.zeros((grid_x, grid_y, grid_z))
    objectives = []

    # 产生多个立方柱 yoz平面
    for i in range(obstacles):
        ry = randint(0, grid_y - thickness - 1)
        rz = randint(0, grid_z - thickness - 1)
        for j in range(grid_x - 1):
            for k in range(ry, ry + thickness):
                for l in range(rz, rz + thickness):
                    occ_grid[j][k][l] = 1

    # 产生多个立方柱 xoz平面
    for i in range(obstacles):
        rx = randint(0, grid_y - thickness - 1)
        rz = randint(0, grid_z - thickness - 1)
        for j in range(grid_y - 1):
            for k in range(rx, rx + thickness):
                for l in range(rz, rz + thickness):
                    occ_grid[k][j][l] = 1

    # 产生多个立方柱 xoy平面
    for i in range(obstacles):
        rx = randint(0, grid_x - thickness - 1)
        ry = randint(0, grid_y - thickness - 1)
        for j in range(grid_x - 1):
            for k in range(rx, rx + thickness):
                for l in range(ry, ry + thickness):
                    occ_grid[k][l][j] = 1

    while len(objectives) < goals:
        x = randint(0, grid_x - 1)
        y = randint(0, grid_y - 1)
        z = randint(0, grid_z - 1)
        if occ_grid[x][y][z] == 0:
            occ_grid[x][y][z] = 1
            objectives.append(Point(x, y, z))
    for i in range(goals):
        occ_grid[objectives[i].x][objectives[i].y][objectives[i].z] = 0

    s_p = Point(randint(0, grid_x - 1), randint(0, grid_z - 1), randint(0, grid_z - 1))
    while occ_grid[s_p.x][s_p.y][s_p.z] > 0:
        s_p = Point(randint(0, grid_x - 1), randint(0, grid_z - 1), randint(0, grid_z - 1))

    return occ_grid, objectives, s_p
'''
