#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
from Point2 import Point
import numpy as np
from mapTools import privacy_init, hasprivacythreat2, initialmapwithknowngrid
from Configure import configure
import math
import sys
from heapq import heappush

sys.setrecursionlimit(1000000)


class AStar:


    class Node:
        def __init__(self, point, endPoint, ideallength, g=0):
            self.point = point
            self.father = None
            self.g = g
            self.step = 0
            self.cam = 0
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y) + abs(endPoint.z - point.z))

        def __str__(self):
            return "point as the node: x:" + str(self.point.x) + ",y:" + str(self.point.y) + ",z:" + str(
                self.point.z) + ",ca:" + str(self.point.ca)


        def __lt__(self, other):
            return self.g + self.h < other.g + other.h

    def __init__(self, occ_grid, pri_grid, grid, sum_privacy, startPoint, endPoint, passTag, Tbudget, threat_list, Toptimal):


        self.openList = []

        self.closeList = []

        self.map3d = occ_grid
        self.grid = grid
        self.prigrid = pri_grid
        self.sumpri = sum_privacy
        self.ideallength = abs(endPoint.x - startPoint.x) + abs(endPoint.y - startPoint.y) + abs(
            endPoint.z - startPoint.z)
        self.Tbudget = Tbudget
        # print("Time limit: ", self.Tbudget)
        self.threatlist = threat_list
        self.timestep = 0
        self.Toptimal = Toptimal
        # self.startPoint = startPoint


        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)


        self.endPoint2 = Point(endPoint.x, endPoint.y, endPoint.z, 1 - endPoint.ca)


        self.passTag = 1

        # print("endpoint",self.endPoint, self.endPoint2)

    """new function"""

    def updateNodeHvalue(self):
        for i in range(len(self.openList)):
            node = self.openList[i]
            # print("#######",node)
            # print("$$$", node.point.x)
            # rou1 = (abs(node.point.x - self.startPoint.x) +
            #        abs(node.point.y - self.startPoint.y) +
            #        abs(node.point.z - self.startPoint.z)) / self.ideallength
            rou1 = 1 - (abs(node.point.x - self.endPoint.x) +
                        abs(node.point.y - self.endPoint.y) +
                        abs(node.point.z - self.endPoint.z)) / self.ideallength
            rou2 = node.step / self.Tbudget
            adaptive1 = math.exp(1 - rou1 / rou2)
            adaptive2 = math.exp(rou1 / rou2 - 1)
            # print(rou1,rou2, adaptive1, adaptive2)
            fathernode = node.father
            # print("*******", node.point.x, fathernode.point.x)
            delta_h = 0
            for j in range(len(self.threatlist)):
                # far away, oppisite
                threat = self.threatlist[j]
                # print(threat)
                """
                f (abs(node.point.x - threat[0]) + abs(node.point.y - threat[1]) + abs(node.point.z - threat[2])) > (
                        abs(self.endPoint.x - threat[0]) + abs(self.endPoint.y - threat[1]) +
                        abs(self.endPoint.z - threat[2])):
                    delta_h += adaptive1 * self.map3d[threat[0]][threat[1]][threat[2]]
                else:
                    delta_h += adaptive2 * self.map3d[threat[0]][threat[1]][threat[2]]
                """
                if (abs(node.point.x - threat[0]) + abs(node.point.y - threat[1]) + abs(node.point.z - threat[2])) > (
                        abs(fathernode.point.x - threat[0]) + abs(fathernode.point.y - threat[1]) +
                        abs(fathernode.point.z - threat[2])):
                    delta_h += adaptive1 * self.map3d[threat[0]][threat[1]][threat[2]]
                else:
                    delta_h += adaptive2 * self.map3d[threat[0]][threat[1]][threat[2]]
            node.h = (abs(self.endPoint.x - node.point.x) + abs(self.endPoint.y - node.point.y) + abs(
                self.endPoint.z - node.point.z)) / self.ideallength
            node.h = node.h * delta_h
            # print("node.h:", node.h)

    # """
    def getMinNode(self):


        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    # """
    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def point_in_close_list(self, point, step_num):
        for node in self.closeList:
            if node.point == point and node.step == step_num:
                return True
        return False

    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None

    def the_same_points_in_open_list(self, point):
        same_points_list = []
        for node in self.openList:
            if node.point == point:
                same_points_list.append(node)
        return same_points_list

    def endPointInCloseList(self):
        for node in self.closeList:  ## 0615
            if node.point == self.endPoint or node.point == self.endPoint2:
                return node
        return None

    def endPointInOpenList(self):
        for node in self.openList:
            if node.point == self.endPoint or node.point == self.endPoint2:  # 0615
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY, offsetZ, cam):


        if (minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.grid[0] - 1 or
                minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.grid[1] - 1 or
                minF.point.z + offsetZ < 0 or minF.point.z + offsetZ > self.grid[2] - 1):
            return

        # if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] != self.passTag:
        # if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] in self.passTag:
        # print("$$$$$$$$$", currentPoint, minF, minF.step)
        if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] == self.passTag:
            return

        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY, minF.point.z + offsetZ, cam)
        # print("##########", currentPoint, minF, minF.step)
        # if self.pointInCloseList(currentPoint):
        #     return
        if self.point_in_close_list(currentPoint, minF.step + 1):
            return

        """new setting for time limit"""
        # print()
        if minF.step + 1 > self.Tbudget:
            # print("time limit", minF, minF.step, currentPoint )
            return


        # step = 1/self.ideallength
        step = 1

        if self.sumpri == 0:
            privacy_threat = 0
        else:
            privacy_threat = (self.prigrid[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] * math.exp(-(cam)) + 1/2 )
        cam_off = cam

        time_punishment = 1
        if minF.step + 1 > self.Toptimal:
            time_punishment = math.exp((minF.step + 1 -self.Toptimal)/(self.Tbudget-self.Toptimal))
        delta_g = time_punishment * step + privacy_threat

        # delta_g = step + privacy_threat

        same_point_list = self.the_same_points_in_open_list(currentPoint)
        if not same_point_list:
            # print("currentPoint:", currentPoint, currentNode)
            currentNode = AStar.Node(currentPoint, self.endPoint, self.ideallength, g=minF.g + delta_g)
            currentNode.father = minF
            currentNode.cam = minF.cam + cam
            currentNode.step = minF.step + 1
            self.openList.append(currentNode)

            # print("MinF$$$$$: ", minF.step, minF.point, currentNode.step, currentNode.point)
            return

        smallest_step_num = self.Tbudget
        same_step_in_list = False
        same_node = None
        for node in same_point_list:
            if minF.step + 1 == node.step:
                same_step_in_list = True
                same_node = node
                break
            if smallest_step_num > node.step:
                smallest_step_num = node.step
        if same_step_in_list:
            if minF.g + delta_g < same_node.g:
                same_node.g = minF.g + delta_g
                same_node.father = minF
                same_node.step = minF.step + 1
                same_node.cam = minF.cam
        else:
            if minF.step + 1 < smallest_step_num:
                currentNode = AStar.Node(currentPoint, self.endPoint, self.ideallength, g=minF.g + delta_g)
                currentNode.father = minF
                currentNode.cam = minF.cam + cam
                currentNode.step = minF.step + 1
                # self.openList.append(currentNode)
                heappush(self.openList, currentNode)
    def start(self):

        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] != self.passTag:
        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] in self.passTag:
        if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] == self.passTag:
            return None

        startNode = AStar.Node(self.startPoint, self.endPoint, self.ideallength)
        #self.openList.append(startNode)
        heappush(self.openList, startNode)

        while True:

            minF = self.getMinNode()
            #print("minF: ", minF.point, minF.step)
            if minF == None :
                print("no solution for minF!")
                return None
            #minF = None
            #if len(self.openList) == 0:
            #    print("No solution for minF!")
            #    return None
            #else:
            #    minF = self.openList[0]

            self.closeList.append(minF)
            self.openList.remove(minF)


            # turn on camera
            # actions = [[0, -1, 0, 0],[0, 1, 0, 0],[-1, 0, 0, 0],[1, 0, 0, 0],[0, 0, 1, 0],[0, 0, -1, 0],[0, -1, 0, 1],[0, 1, 0, 1],[-1, 0, 0, 1],[0, 0, 1, 1],[0, 0, -1, 1],[1, 0, 0, 1]]
            # actionlist = [0,1,2,3,4,5,6,7,8,9,10,11]
            # random.shuffle(actionlist)

            # for i in range (len(actionlist)):
            #    self.searchNear(minF, actions[actionlist[i]][0], actions[actionlist[i]][1], actions[actionlist[i]][2], actions[actionlist[i]][3])
            # """
            # turn on camera
            self.searchNear(minF, 0, -1, 0, 0)
            self.searchNear(minF, 0, 1, 0, 0)
            self.searchNear(minF, -1, 0, 0, 0)
            self.searchNear(minF, 1, 0, 0, 0)
            self.searchNear(minF, 0, 0, 1, 0)
            self.searchNear(minF, 0, 0, -1, 0)


            self.updateNodeHvalue()


            point = self.endPointInCloseList()
            if point:
                print("The plan found!")
                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        # print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())
                        return list(reversed(pathList))
            if len(self.openList) == 0:
                print("No plan could meet the time limit!")
                return None



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
    T_optimal = config.T_optimal
    viewradius = config.viewradius
    Kca = config.Kca
    threat_list = []
    reinitial_flag = 0

    occ_grid = np.load(file="occ_grid.npy")
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    if reinitial_flag:
        occ_grid_known, pri_grid_known, privacy_sum_known = initialmapwithknowngrid(grid_x, grid_y, grid_z,
                                                                               privacy_threshold, privacy_radius,
                                                                               occ_grid)
    else:

        occ_grid_known = np.load(file="occ_grid_known.npy")
        pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)



    print("The occ_grid is: ")
    for m in range(grid_x):
        print("The value of x: ", m)
        print(occ_grid[m])
    starttime = time.time()
    aStar1 = AStar(occ_grid_known, pri_grid_known, grid, privacy_sum_known, starting_point, end_point, [1], T_budget, threat_list, T_optimal)


    trajectory_ref = aStar1.start()

    aStar2 = AStar(occ_grid, pri_grid, grid, privacy_sum, starting_point, end_point, [1], T_budget, threat_list, T_optimal)


    trajectory_plan = aStar2.start()

    endtime = time.time()
    dtime = endtime - starttime

    trajectory_ref = [starting_point] + trajectory_ref
    trajectory_plan = [starting_point] + trajectory_plan

    refpath = np.zeros((len(trajectory_ref),4))
    planpath = np.zeros((len(trajectory_plan),4))

    for i in range(len(trajectory_ref)):
        refpath[i]=[trajectory_ref[i].x, trajectory_ref[i].y, trajectory_ref[i].z, trajectory_ref[i].ca]

    for i in range(len(trajectory_plan)):
        planpath[i] = [trajectory_plan[i].x,trajectory_plan[i].y, trajectory_plan[i].z, trajectory_plan[i].ca]

    np.save(file="reference_path.npy", arr=refpath)
    b = np.load(file="reference_path.npy")
    print(b, len(b))

    np.save(file="plan_path.npy",arr=planpath)
    c = np.load(file="plan_path.npy")
    print(c,len(c))

    sum = 0
    num_ca = 0
    num_intruder = 0
    for point in trajectory_ref:
        sum += pri_grid[point.x][point.y][point.z] * math.exp(-(point.ca) + 1/2 )
        #if pri_grid[point.x][point.y][point.z] > 0:
        # print(point, pri_grid_known[point.x][point.y][point.z])
    print("\033[94m Fitness for reference path:\033[0m \n", len(trajectory_ref) - 1, sum, num_ca)

    sum = 0
    num_ca = 0
    num_intruder = 0
    for point in trajectory_plan:
        sum += pri_grid[point.x][point.y][point.z] * math.exp(-(point.ca) + 1/2)
        #if pri_grid[point.x][point.y][point.z] > 0:
    # print(point, pri_grid_known[point.x][point.y][point.z])
    print("\033[94m Fitness for replanned path:\033[0m \n", len(trajectory_plan) - 1, sum, num_ca)

    if reinitial_flag:
        np.save(file="occ_grid_known.npy", arr=occ_grid_known)
        c = np.load(file="occ_grid_known.npy")
        #for m in range(grid_x):
        #    print("The value of x: ", m)
        #    print(c[m])
