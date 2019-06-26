#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
add camera into searching space
"""
import time
from point import Point
import numpy as np
from mapTools import privacy_init, hasprivacythreat2, initialmapwithknowngrid
import copy
from Configure import configure
import math
# from quickSort import quick_sort
import sys
from heapq import heappush

# from log import Log
# log = Log(__name__).getlog()

sys.setrecursionlimit(1000000)


class AStar:
    """
    AStar算法的Python3.x实现
    """

    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, ideallength, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.step = 0
            self.cam = 0
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y) + abs(endPoint.z - point.z))/ideallength # 计算h值 曼哈顿距离

        def __str__(self):
            return "point as the node: x:" + str(self.point.x) + ",y:" + str(self.point.y) + ",z:" + str(self.point.z) + ",ca:" + str(self.point.ca)

        # 堆需要节点与节点之间的比较，因此必须实现这个魔术方法
        def __lt__(self, other):
            return self.g+self.h < other.g+other.h

    def __init__(self, occ_grid, pri_grid, grid, sum_privacy, startPoint, endPoint, passTag, Tbudget, threat_list, flag, Toptimal):
        """
        构造AStar算法的启动条件
        :param map3d: Array2D类型的寻路数组
        :param startPoint: Point或二元组类型的寻路起点
        :param endPoint: Point或二元组类型的寻路终点
        :param passTag: int类型的可行走标记（若地图数据!=passTag即为障碍）
        :param threatlist: privacy restricted area affected
        """
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 寻路地图
        self.map3d = occ_grid
        self.grid = grid
        self.prigrid = pri_grid
        self.sumpri = sum_privacy
        self.ideallength = abs(endPoint.x - startPoint.x) + abs(endPoint.y - startPoint.y) + abs(endPoint.z - startPoint.z)
        self.Tbudget = Tbudget
        self.Toptimal = Toptimal
        # print("Time limit: ", self.Tbudget)
        self.threatlist = threat_list
        self.timestep = 0
        #self.startPoint = startPoint
        self.flag = flag


        # 起点终点
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)

        ## 结束点的第二种可能性 - 0615
        self.endPoint2 = Point(endPoint.x, endPoint.y, endPoint.z, 1-endPoint.ca)

        # 障碍物标记
        self.passTag = 1

        #print("endpoint",self.endPoint, self.endPoint2)

    """new function"""

    def updateNodeHvalue(self):
        for i in range(len(self.openList)):
            node = self.openList[i]
            # print("#######",node)
            # print("$$$", node.point.x)
            # rou1 = (abs(node.point.x - self.startPoint.x) +
            #         abs(node.point.y - self.startPoint.y) +
            #         abs(node.point.z - self.startPoint.z)) / self.ideallength
            rou1 = 1- (abs(node.point.x - self.endPoint.x) +
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
                #print(threat)
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
                    delta_h += adaptive1 * self.map3d[threat[0]][threat[1]][threat[2]] ## 绕路
                else:
                    delta_h += adaptive2 * self.map3d[threat[0]][threat[1]][threat[2]]
            node.h = (abs(self.endPoint.x - node.point.x) + abs(self.endPoint.y - node.point.y) + abs(self.endPoint.z - node.point.z))/self.ideallength
            node.h = node.h * delta_h
            # print("node.h:", node.h)

    # def getMinNode(self):
    #     """
    #     获得openlist中F值最小的节点
    #     :return: Node
    #     """
    #     """
    #     currentNode = self.openList[0]
    #     for node in self.openList:
    #         if node.g + node.h < currentNode.g + currentNode.h:
    #             currentNode = node
    #     return currentNode
    #     """
    #     # currentNode = self.openList[0]
    #     # for node in self.openList:
    #     #     if node.g + node.h < currentNode.g + currentNode.h: ## 0615 why not <=, but <?????
    #     #         if node.step <= self.Tbudget:
    #     #             currentNode = node
    #     # if currentNode.point != self.startPoint:
    #     #     print("MinF: " , currentNode.father.step, currentNode.father.point, currentNode.step, currentNode.point)
    #     # if currentNode.step <= self.Tbudget:
    #     #     return currentNode
    #     # else:
    #     #     return None
    #
    #     quick_sort(self.openList)
    #     self.openList = list(reversed(self.openList))
    #     # currentNode = self.openList[0]
    #     for node in self.openList:
    #         if node.step <= self.Tbudget:
    #             return node
    #     return None

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
        for node in self.closeList: ## 0615
            if node.point == self.endPoint or node.point == self.endPoint2:
                return node
        return None

    def endPointInOpenList(self):
        for node in self.openList:
            if node.point == self.endPoint or node.point == self.endPoint2: # 0615
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY, offsetZ, cam):
        """
        搜索节点周围的点
        :param minF:F值最小的节点
        :param offsetX:坐标偏移量
        :param offsetY:
        :return:
        """
        # 越界检测
        if (minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.grid[0] - 1 or
                minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.grid[1] - 1 or
                minF.point.z + offsetZ < 0 or minF.point.z + offsetZ > self.grid[2] - 1):
            return
        # 如果是障碍，就忽略
        # if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] != self.passTag:
        # if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] in self.passTag:
        # print("$$$$$$$$$", currentPoint, minF, minF.step)
        if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] == self.passTag:
            return
        # 如果在关闭表中，就忽略
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY, minF.point.z + offsetZ, cam)
        # print("##########", currentPoint, minF, minF.step)
        # if self.pointInCloseList(currentPoint):
        #     return
        if self.point_in_close_list(currentPoint, minF.step+1):
            return

        """new setting for time limit"""
        # print()
        if minF.step + 1 > self.Tbudget:
            # print("time limit", minF, minF.step, currentPoint )
            return

        # 设置单位花费

        # step = 1/self.ideallength
        step = 1

        if self.sumpri == 0:
            privacy_threat = 0
        else:
            privacy_threat = (self.prigrid[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] * math.exp(-(cam))+1/2)
        cam_off = cam

        time_punishment = 1
        if minF.step + 1 > self.Toptimal:
            time_punishment = math.exp((minF.step + 1 - self.Toptimal) / (self.Tbudget - self.Toptimal))
        delta_g = time_punishment * step + privacy_threat
        #delta_g = step + cam_off + privacy_threat

        # 如果不在openList中，就把它加入openlist
        # currentNode = self.pointInOpenList(currentPoint)
        # 用一个列表来收集相同的点
        same_point_list = self.the_same_points_in_open_list(currentPoint)
        if not same_point_list:
            # print("currentPoint:", currentPoint, currentNode)
            currentNode = AStar.Node(currentPoint, self.endPoint, self.ideallength, g=minF.g + delta_g)
            currentNode.father = minF
            currentNode.cam = minF.cam + cam
            currentNode.step = minF.step + 1
            # self.openList.append(currentNode)
            heappush(self.openList, currentNode)

            #print("MinF$$$$$: ", minF.step, minF.point, currentNode.step, currentNode.point)
            return
        # # 如果在openList中，判断minF到当前点的G是否更小
        # if minF.g + delta_g < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
        #     currentNode.g = minF.g + delta_g
        #     currentNode.father = minF
        #     currentNode.step = minF.step + 1
        #     currentNode.cam = minF.cam
        #     # print("MinF#####: ", currentNode.father.step, currentNode.father.point, currentNode.step, currentNode.point)

        # 检查这些相同的点的step值，如果有相同的，就更新相同的
        # 如果没有相同的，就看看当前的step值和最小的step值
        # 如果最小的step值小，说明当前的step没用
        # 如果当前的step小，说明当前的step值有用，添加到openlist中
        smallest_step_num = self.Tbudget
        same_step_in_list = False
        same_node = None
        for node in same_point_list:
            if minF.step+1 == node.step:
                same_step_in_list = True
                same_node = node
                break
            if smallest_step_num > node.step:
                smallest_step_num = node.step
        if same_step_in_list:
            if minF.g + delta_g < same_node.g:  # 如果更小，就重新计算g值，并且改变father
                same_node.g = minF.g + delta_g
                same_node.father = minF
                same_node.step = minF.step + 1
                same_node.cam = minF.cam
        else:
            if minF.step+1 < smallest_step_num:
                currentNode = AStar.Node(currentPoint, self.endPoint, self.ideallength, g=minF.g + delta_g)
                currentNode.father = minF
                currentNode.cam = minF.cam + cam
                currentNode.step = minF.step + 1
                # self.openList.append(currentNode)
                heappush(self.openList, currentNode)

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 判断寻路终点是否是障碍
        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] != self.passTag:
        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] in self.passTag:
        if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] == self.passTag:
            return None
        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint, self.ideallength)
        # self.openList.append(startNode)
        heappush(self.openList, startNode)
        # 2.主循环逻辑
        while True:
            # 找到F值最小的点
            # minF = self.getMinNode()
            # # print("minF: ", minF.point, minF.step)
            # if minF == None :
            #     print("no solution for minF!")
            #     return None
            minF = None
            if len(self.openList) == 0:
                print("No solution for minF!")
                return None
            else:
                minF = self.openList[0]
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)

            # 判断这个节点的上下左右节点
            # turn on camera
            # actions = [[0, -1, 0, 0],[0, 1, 0, 0],[-1, 0, 0, 0],[1, 0, 0, 0],[0, 0, 1, 0],[0, 0, -1, 0],[0, -1, 0, 1],[0, 1, 0, 1],[-1, 0, 0, 1],[0, 0, 1, 1],[0, 0, -1, 1],[1, 0, 0, 1]]
            # actionlist = [0,1,2,3,4,5,6,7,8,9,10,11]
            # random.shuffle(actionlist)

            # for i in range (len(actionlist)):
            #    self.searchNear(minF, actions[actionlist[i]][0], actions[actionlist[i]][1], actions[actionlist[i]][2], actions[actionlist[i]][3])
            # """
            # turn on camera
            if self.flag == 0:
                self.searchNear(minF, 0, -1, 0, 0)
                self.searchNear(minF, 0, 1, 0, 0)
                self.searchNear(minF, -1, 0, 0, 0)
                self.searchNear(minF, 1, 0, 0, 0)
                self.searchNear(minF, 0, 0, 1, 0)
                self.searchNear(minF, 0, 0, -1, 0)
            else:
                self.searchNear(minF, 0, -1, 0, 0)
                self.searchNear(minF, 0, 1, 0, 0)
                self.searchNear(minF, -1, 0, 0, 0)
                self.searchNear(minF, 1, 0, 0, 0)
                self.searchNear(minF, 0, 0, 1, 0)
                self.searchNear(minF, 0, 0, -1, 0)
                self.searchNear(minF, 0, -1, 0, 1)
                self.searchNear(minF, 0, 1, 0, 1)
                self.searchNear(minF, -1, 0, 0, 1)
                self.searchNear(minF, 1, 0, 0, 1)
                self.searchNear(minF, 0, 0, 1, 1)
                self.searchNear(minF, 0, 0, -1, 1)

            self.updateNodeHvalue()

            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                # print("The plan found!")
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

def Astar_Hybrid_Planning_online(config, iteration, log):

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
    replantime = 0

    occ_grid_name = "occ_grid" + str(iteration) + ".npy"
    occ_grid = np.load(file=occ_grid_name)
    # occ_grid = np.load(file="occ_grid.npy")
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)
    occ_grid_known_name = "occ_grid_known" + str(iteration) + ".npy"
    occ_grid_known = np.load(file=occ_grid_known_name)
    # occ_grid_known = np.load(file="occ_grid_known.npy")
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)

    # print("The occ_grid is: ")
    # for m in range(grid_x):
    #     print("The value of x: ", m)
    #     print(occ_grid[m])
    starttime = time.time()
    # aStar = AStar(occ_grid, pri_grid_known, grid, privacy_sum_known, starting_point, end_point, [1], T_budget, threat_list, 0)
    # 开始寻路
    # trajectory_ref = aStar.start()
    reference_path_name = "reference_path" + str(iteration) + ".npy"
    trajectory_ref_temp = np.load(file=reference_path_name)
    # trajectory_ref_temp = np.load(file="reference_path.npy")
    # trajectory_ref_temp = np.load(file="plan_path_Hybrid.npy")
    trajectory_ref = []
    for i in range(len(trajectory_ref_temp)):
        point = Point(int(trajectory_ref_temp[i][0]), int(trajectory_ref_temp[i][1]), int(trajectory_ref_temp[i][2]),
                      int(trajectory_ref_temp[i][3]))
        # if (pri_grid_known[point.x][point.y][point.z] > 0):
        #     print(point)
        trajectory_ref.append(point)

    endtime = time.time()
    dtime = endtime - starttime
    # print("程序运行时间：%.8s s" % dtime)

    path_grid = copy.deepcopy(occ_grid)

    sum = 0
    if trajectory_ref == None:
        print("No solution!")
        exit(0)
    else:
        for point in trajectory_ref:
            # for ii in range(len(trajectory_ref)):
            # print(point)
            path_grid[point.x][point.y][point.z] = 9
            sum += pri_grid_known[point.x][point.y][point.z]
            # print(point, pri_grid_known[point.x][point.y][point.z])
        # print("----------------------\n", len(trajectory_ref))

    # 再次显示地图

    # print(path_grid, sum)
    # trajectory_ref = [starting_point] + trajectory_ref
    trajectory_plan = copy.deepcopy(trajectory_ref)
    # sensor_initial = np.zeros(len(trajectory_plan))
    # sensor_plan = copy.deepcopy(sensor_initial)
    time_step = 0
    # print("---------------------------------")
    # print("The length of original plan is: ", len(trajectory_plan))
    # for m in range(len(trajectory_plan)):
    #     print("The No.", m, " step: ", trajectory_plan[m])
    # print()

    idx = 0
    current_f = sum + len(trajectory_plan)

    while not (idx >= len(trajectory_plan)):
        current_p = trajectory_plan[idx]
        current_ca = trajectory_plan[idx].ca

        if current_p.x == end_point.x and current_p.y == end_point.y and current_p.z == end_point.z:
            break

        next_p = trajectory_plan[idx + 1]
        next_idx = idx + 1
        # print("The UAV would move a step: ")
        # print("The current point: ", current_p)
        # print("The next point: ", next_p)
        # print("The index of next point: ", next_idx, "\n")

        if current_ca == 1:
            time_step += 1
            current_p = next_p
            idx += 1
            # print("The UAV has finished this step.\n")
            continue

        # take picture
        # update occ_grid, pri_grid
        flag, occ_grid_known, pri_grid_known, privacy_sum_known, threat_list = hasprivacythreat2(current_p,
                                                                                                 occ_grid_known,
                                                                                                 occ_grid,
                                                                                                 pri_grid_known,
                                                                                                 privacy_sum_known,
                                                                                                 config)
        # print("The length of privacy_list: ", len(threat_list))
        # for m in range(len(threat_list)):
        #     print(threat_list[m])

        if flag:
            ## 0622
            for j in range(idx + 1, len(trajectory_plan)):
                if (pri_grid_known[trajectory_plan[j].x][trajectory_plan[j].y][trajectory_plan[j].z] * math.exp(
                        -(trajectory_plan[j].ca) + 1 / 2)) > 0:
                    next_p = trajectory_plan[j]
                    next_idx = j
                else:
                    break

            # print("---------------------------------")
            # print("The UAV produce a temporory plan!")
            # print("The index of current point: ", idx)
            # print("The current point: ", current_p)
            # print("The index of next point: ", next_idx)
            # print("The next point: ", next_p, "\n")

            # if  (next_idx == idx + 1)  and (pri_grid_known[trajectory_plan[next_idx].x][trajectory_plan[next_idx].y][trajectory_plan[next_idx].z] > 0) :
            #    trajectory_plan[next_idx].ca = 1
            #    print ("change sensor configuration for next point")

            if next_idx != idx + 1:  # no need for motion planning

                """删除冗余路径"""
                if current_p.x == next_p.x and current_p.y == next_p.y and current_p.z == next_p.z: # 0623
                # if current_p == next_p:
                    # print("delete redundant route", current_p, next_p)
                    first_part = trajectory_plan[:idx]
                    next_part = trajectory_plan[next_idx + 1:]
                    trajectory_plan = first_part + next_part
                    break

                # print("The length of trajectory_plan: ", len(trajectory_plan))
                T_plan = T_budget - (len(trajectory_plan) - 1) + (next_idx - idx)
                T_plan_optimal = T_optimal - (len(trajectory_plan) - 1) + (next_idx - idx)
                # print("The time limit: ", T_plan, "\n")
                # if T_plan < (abs(trajectory_plan.points[next_idx].x - trajectory_plan.points[idx].x) + abs(trajectory_plan.points[next_idx].y - trajectory_plan.points[idx].y) + \
                #        abs(trajectory_plan.points[next_idx].z - trajectory_plan.points[idx].z)):
                #    print("no solution!")
                # print(T_plan, current_p,  next_p)

                distance = abs(trajectory_plan[next_idx].x - trajectory_plan[idx].x) + abs(
                    trajectory_plan[next_idx].y - trajectory_plan[idx].y) + abs(
                    trajectory_plan[next_idx].z - trajectory_plan[idx].z)
                ## have enough time for planning
                if T_plan >= distance:
                    # 开始寻路
                    start1 = time.time()
                    aStar = AStar(occ_grid, pri_grid_known, grid, privacy_sum_known, current_p, next_p, [1], T_plan,
                                  threat_list, 1, T_plan_optimal)

                    # print('\033[94m finding solution for local planning... \033[0m')
                    trajectory_optimal = aStar.start()
                    end1 = time.time()
                    dtime = end1 - start1
                    # print("程序运行时间：%.8s s" % dtime)
                    if trajectory_optimal == None:
                        # print('\033[94mNo solution for local planning... \033[0m')
                        # print()
                        for kk in range(idx + 1, next_idx + 1):
                            trajectory_plan[kk].ca = 1

                    else:
                        # print("The length of local planning: ", len(trajectory_optimal))
                        # print()
                        previous_trajectory = copy.deepcopy(trajectory_plan[:idx])
                        following_trajectory = copy.deepcopy(trajectory_plan[next_idx + 1:])

                        now_trajectory = []
                        first_part = trajectory_plan[0:idx + 1]
                        following_part = trajectory_plan[next_idx + 1:]
                        now_trajectory = first_part + trajectory_optimal + following_part

                        replan_flag = 0
                        for m in range(idx + 1, next_idx + 1):
                            # print("original， The No.", m, " step: ", trajectory_plan[m])
                            if (len(trajectory_optimal) != (next_idx - idx)):
                                replan_flag = 1
                                break
                            if (trajectory_plan[m] != trajectory_optimal[m - idx - 1]):
                                replan_flag = 1

                        if replan_flag:
                            replantime += 1  ## 排除重复规划的相同路径 0620

                        # for m in range(idx + 1, next_idx + 1):
                            # print("original， The No.", m, " step: ", trajectory_plan[m])

                        # print("The length of now plan is: ", len(trajectory_optimal))
                        # for m in range(len(trajectory_optimal)):
                        #     print("trajectory_optimal， The No.", m, " step: ", trajectory_optimal[m])
                        #
                        # for m in range(len(now_trajectory)):
                        #     print("The No.", m, " step: ", now_trajectory[m])
                        # print()
                        """
                        for ll in range(idx+1):
                            temp = Point(trajectory_plan[ll].x, trajectory_plan[ll].y,
                                 trajectory_plan[ll].z, trajectory_plan[ll].ca)
                            now_trajectory.append(temp)

                        for ll in range(0, len(trajectory_optimal)):
                            temp = Point(trajectory_optimal[ll].x,trajectory_optimal[ll].y,trajectory_optimal[ll].z,trajectory_optimal[ll].ca)
                            now_trajectory.append(temp)


                        for ll in range(next_idx+1,len(trajectory_plan)):
                            temp = Point(trajectory_plan[ll].x,trajectory_plan[ll].y,trajectory_plan[ll].z,trajectory_plan[ll].ca)
                            now_trajectory.append(temp)
                        """

                        trajectory_plan = copy.deepcopy(now_trajectory)
                        # print("The UAV would move a step: ")
                        # print("The current point: ", current_p)
                        # print("The next point: ", trajectory_plan[idx + 1])
                        # print("The index of next point: ", idx + 1, "\n")
                # turn off camera never exist
                else:
                    # print("sensor reconfigured for the next points in the path!!!")
                    for kk in range(idx + 1, next_idx + 1):
                        trajectory_plan[kk].ca = 1
                sum = 0
                cam_off = 0
                for ll in range(len(trajectory_plan)):
                    sum += pri_grid_known[trajectory_plan[ll].x][trajectory_plan[ll].y][trajectory_plan[ll].z]
                    cam_off += trajectory_plan[ll].ca
                    # print("now", trajectory_plan[ll])
                # print("The length of now_trajectory_plan: ", len(trajectory_plan), sum, cam_off)

                current_f = sum + len(trajectory_plan) + cam_off

                # print("fitness", current_f)

            elif pri_grid_known[trajectory_plan[next_idx].x][trajectory_plan[next_idx].y][
                trajectory_plan[next_idx].z] > 0:
                trajectory_plan[next_idx].ca = 1
                # print("change sensor configuration for next point")

        time_step += 1
        idx = idx + 1
        # print("The UAV has finished this step.\n")

        # if idx == 4:
        #    print("the debug begin!")
        # print("next point", idx,time_step, len(trajectory_plan))
    # print(occ_grid)
    path_grid2 = copy.deepcopy(occ_grid)
    sum_plan = 0
    num_ca_plan = 0
    num_intruder_plan = 0
    for point in trajectory_plan:
        if point.ca == 0:
            path_grid2[point.x][point.y][point.z] = 7
        else:
            path_grid2[point.x][point.y][point.z] = 10
            num_ca_plan += 1
        sum_plan += pri_grid[point.x][point.y][point.z] * math.exp(-(point.ca) + 1 / 2)
        if pri_grid[point.x][point.y][point.z] > 0:
            num_intruder_plan += 1
        # print(point, pri_grid_known[point.x][point.y][point.z])
    print("\033[94mFitness for replanned path:\033[0m \n ", len(trajectory_plan) - 1, sum_plan, num_ca_plan,
          num_intruder_plan)
    log.info("Online_Hybrid_Planning: Length of replanned trajectory: %d" %(len(trajectory_plan) - 1))
    log.info("Online_Hybrid_Planning: Sum of privacy threat of replanned trajectory: %f" % sum_plan)
    log.info("Online_Hybrid_Planning: Times of turning off camera of replanned trajectory: %d" % num_ca_plan)
    log.info("Online_Hybrid_Planning: Times of intrusion of replanned trajectory: %d" % num_intruder_plan)

    # 再次显示地图
    sum_ref = 0
    num_ca_ref = 0
    num_intruder_ref = 0
    for point in trajectory_ref:
        sum_ref += pri_grid[point.x][point.y][point.z] * math.exp(-(point.ca) + 1 / 2)
        num_ca_ref += point.ca
        # print(point, pri_grid_known[point.x][point.y][point.z])
        if pri_grid[point.x][point.y][point.z] > 0:
            num_intruder_ref += 1
    print("\033[94m Fitness for reference path:\033[0m \n", len(trajectory_ref) - 1, sum_ref, num_ca_ref,
          num_intruder_ref)
    log.info("Online_Hybrid_Planning: Length of preplanned trajectory: %d" % (len(trajectory_ref) - 1))
    log.info("Online_Hybrid_Planning: Sum of privacy threat of preplanned trajectory: %f" % sum_ref)
    log.info("Online_Hybrid_Planning: Times of turning off camera of preplanned trajectory: %d" % num_ca_ref)
    log.info("Online_Hybrid_Planning: Times of intrusion of preplanned trajectory: %d" % num_intruder_ref)

    # 再次显示地图

    # print(path_grid2, sum)
    # print("---------------------------------")
    # print("The last plan is finished!")
    # print("The length of last plan is: ", len(trajectory_plan))
    # for m in range(len(trajectory_plan)):
    #     print("The No.", m, " step: ", trajectory_plan[m])
    end = time.time()
    dtime = end - starttime
    # print("程序运行时间：%.8s s" % dtime)
    # print("sumpri:", sum)
    # print("num_ca:", num_ca)
    print("\033[94m Replan times: \033[0m", replantime)
    log.info("Online_Hybrid_Planning: Replanning times: %d" % replantime)
    # grid_visualization(occ_grid, starting_point, end_point, trajectory_plan, trajectory_ref)

    occ_grid_known_name = "occ_grid_known" + str(iteration) + ".npy"
    np.save(file=occ_grid_known_name, arr=occ_grid_known)
    # np.save(file="occ_grid_known.npy", arr=occ_grid_known)
    # b = np.load(file="occ_grid_known.npy")
    # for m in range(grid_x):
    #     print("The value of x: ", m)
    #     print(b[m])

    plan_path = np.zeros((len(trajectory_plan), 4))
    for i in range(len(trajectory_plan)):
        plan_path[i] = [trajectory_plan[i].x, trajectory_plan[i].y, trajectory_plan[i].z, trajectory_plan[i].ca]

    plan_path_Hybrid_name = "plan_path_Hybrid" + str(iteration) + ".npy"
    np.save(file=plan_path_Hybrid_name, arr=plan_path)
    # np.save(file="plan_path_Hybrid.npy", arr=plan_path)
    # c = np.load(file="plan_path_Hybrid.npy")
    # print(c, len(c))

    exploration_rate = 0

    for i in range(grid_x):
        for j in range(grid_y):
            for k in range(grid_z):
                if occ_grid_known[i][j][k] != occ_grid[i][j][k]:
                    exploration_rate += 1
    exploration_rate = 1 - exploration_rate / (grid_x * grid_y * grid_z * privacy_threshold)
    print("\033[94m exploration rate: \033[0m", exploration_rate)
    log.info("Online_Hybrid_Planning: Exploration rate: %f" % exploration_rate)

    return sum_plan, len(trajectory_plan)-1, num_intruder_plan, sum_ref, len(trajectory_ref) - 1, num_intruder_ref


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
    replantime = 0

    #occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap(grid_x, grid_y, grid_z,
    #                                                                                       starting_point, end_point,
    #                                                                                       safety_threshold,
    #                                                                                       privacy_threshold,
    #                                                                                       privacy_radius)
    #occ_grid = np.load(file="occ_grid.npy")
    #occ_grid_known, pri_grid_known, privacy_sum_known = initialmapwithknowngrid(grid_x, grid_y, grid_z, privacy_threshold, privacy_radius, occ_grid)
    #pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    occ_grid = np.load(file="occ_grid.npy")
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)
    occ_grid_known = np.load(file="occ_grid_known.npy")
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    # print(pri_grid_known)

    # print("The occ_grid is: ")
    # for m in range(grid_x):
    #     print("The value of x: ", m)
    #     print(occ_grid[m])
    starttime = time.time()
    # aStar = AStar(occ_grid, pri_grid_known, grid, privacy_sum_known, starting_point, end_point, [1], T_budget, threat_list, 0)
    # 开始寻路
    #trajectory_ref = aStar.start()
    trajectory_ref_temp = np.load(file="reference_path.npy")
    #trajectory_ref_temp = np.load(file="plan_path_Hybrid.npy")
    trajectory_ref = []
    for i in range (len(trajectory_ref_temp)):
        point = Point(int(trajectory_ref_temp[i][0]),int(trajectory_ref_temp[i][1]),int(trajectory_ref_temp[i][2]),int(trajectory_ref_temp[i][3]))
        # if(pri_grid_known[point.x][point.y][point.z]>0):
            # print(point)
        trajectory_ref.append(point)

    endtime = time.time()
    dtime = endtime - starttime
    # print("程序运行时间：%.8s s" % dtime)

    path_grid = copy.deepcopy(occ_grid)


    sum = 0
    if trajectory_ref == None:
        print("No solution!")
        exit(0)
    else:
        for point in trajectory_ref:
        #for ii in range(len(trajectory_ref)):
            #print(point)
            path_grid[point.x][point.y][point.z] = 9
            sum += pri_grid_known[point.x][point.y][point.z]
            # print(point, pri_grid_known[point.x][point.y][point.z])
        #print("----------------------\n", len(trajectory_ref))

    # 再次显示地图

    # print(path_grid, sum)
    #trajectory_ref = [starting_point] + trajectory_ref
    trajectory_plan = copy.deepcopy(trajectory_ref)
    # sensor_initial = np.zeros(len(trajectory_plan))
    # sensor_plan = copy.deepcopy(sensor_initial)
    time_step = 0
    print("---------------------------------")
    print("The length of original plan is: ", len(trajectory_plan))
    for m in range(len(trajectory_plan)):
        print("The No.", m, " step: ", trajectory_plan[m])
    print()

    idx = 0
    current_f = sum + len(trajectory_plan)


    while not (idx >= len(trajectory_plan)):
        current_p = trajectory_plan[idx]
        current_ca = trajectory_plan[idx].ca

        if current_p.x == end_point.x and current_p.y == end_point.y and current_p.z == end_point.z :
            break

        next_p = trajectory_plan[idx+1]
        next_idx = idx + 1
        # print("The UAV would move a step: ")
        # print("The current point: ", current_p)
        # print("The next point: ", next_p)
        # print("The index of next point: ", next_idx, "\n")

        if current_ca == 1:
            time_step += 1
            current_p = next_p
            idx += 1
            # print("The UAV has finished this step.\n")
            continue

        # take picture
        # update occ_grid, pri_grid
        flag, occ_grid_known, pri_grid_known, privacy_sum_known, threat_list = hasprivacythreat2 (current_p, occ_grid_known, occ_grid, pri_grid_known, privacy_sum_known, config)
        # print("The length of privacy_list: ", len(threat_list))
        for m in range(len(threat_list)):
            print(threat_list[m])

        if flag:
            # localization
            # p_threat, h_impact = privacy_modeling()
            # update occ_grid, pri_grid

            # for j in range (idx+1, len(trajectory_plan)):
            #     sigma_privacy = 0
            #     for k in range (j,len(trajectory_plan)):
            #         sigma_privacy += pri_grid_known[trajectory_plan[k].x][trajectory_plan[k].y][trajectory_plan[k].z]* math.exp(-(trajectory_plan[k].ca) + 1/2)
            #     if sigma_privacy == 0:
            #         next_p = trajectory_plan[j]
            #         next_idx = j
            #         break
            #     elif k == len(trajectory_plan)-1 :
            #         next_p = trajectory_plan[-1]
            #         next_idx = len(trajectory_plan)-1

            ## 0622
            for j in range (idx+1, len(trajectory_plan)):
                if (pri_grid_known[trajectory_plan[j].x][trajectory_plan[j].y][trajectory_plan[j].z]* math.exp(-(trajectory_plan[j].ca) + 1/2)) > 0:
                    next_p = trajectory_plan[j]
                    next_idx = j
                else:
                    break

            # print("---------------------------------")
            # print("The UAV produce a temporory plan!")
            # print("The index of current point: ", idx)
            # print("The current point: ", current_p)
            # print("The index of next point: ", next_idx)
            # print("The next point: ", next_p, "\n")

            # if  (next_idx == idx + 1)  and (pri_grid_known[trajectory_plan[next_idx].x][trajectory_plan[next_idx].y][trajectory_plan[next_idx].z] > 0) :
            #    trajectory_plan[next_idx].ca = 1
            #    print ("change sensor configuration for next point")

            if next_idx != idx + 1:  # no need for motion planning

                """删除冗余路径"""
                if current_p == next_p:
                    # print("delete redundant route", current_p, next_p)
                    first_part = trajectory_plan[:idx]
                    next_part = trajectory_plan[next_idx+1:]
                    trajectory_plan = first_part + next_part
                    break

                # print("The length of trajectory_plan: ", len(trajectory_plan))
                T_plan = T_budget - (len(trajectory_plan)-1) + (next_idx - idx)
                T_plan_optimal = T_optimal - (len(trajectory_plan)-1) + (next_idx - idx)
                # print("The time limit: ", T_plan, "\n")
                #if T_plan < (abs(trajectory_plan.points[next_idx].x - trajectory_plan.points[idx].x) + abs(trajectory_plan.points[next_idx].y - trajectory_plan.points[idx].y) + \
                #        abs(trajectory_plan.points[next_idx].z - trajectory_plan.points[idx].z)):
                #    print("no solution!")
                # print(T_plan, current_p,  next_p)

                distance = abs(trajectory_plan[next_idx].x-trajectory_plan[idx].x) + abs(trajectory_plan[next_idx].y-trajectory_plan[idx].y) + abs(trajectory_plan[next_idx].z-trajectory_plan[idx].z)
                ## have enough time for planning
                if T_plan >= distance:
                    # 开始寻路
                    start1 = time.time()
                    aStar = AStar(occ_grid, pri_grid_known, grid, privacy_sum_known, current_p, next_p, [1], T_plan, threat_list, 1, T_plan_optimal)



                    #print('\033[94m finding solution for local planning... \033[0m')
                    trajectory_optimal = aStar.start()
                    end1 = time.time()
                    dtime = end1 - start1
                    # print("程序运行时间：%.8s s" % dtime)
                    if trajectory_optimal == None:
                        # print('\033[94mNo solution for local planning... \033[0m')
                        # print()
                        for kk in range(idx + 1, next_idx + 1):
                            trajectory_plan[kk].ca = 1

                    else:
                        # print("The length of local planning: ", len(trajectory_optimal))
                        # print()
                        previous_trajectory = copy.deepcopy(trajectory_plan[ :idx])
                        following_trajectory = copy.deepcopy(trajectory_plan[next_idx+1: ])

                        now_trajectory = []
                        first_part = trajectory_plan[0:idx+1]
                        following_part = trajectory_plan[next_idx+1:]
                        now_trajectory = first_part + trajectory_optimal + following_part

                        replan_flag = 0
                        for m in range(idx+1, next_idx+1):
                            # print("original， The No.", m, " step: ", trajectory_plan[m])
                            if (len(trajectory_optimal)!= (next_idx-idx)):
                                replan_flag = 1
                                break
                            if (trajectory_plan[m]!=trajectory_optimal[m-idx-1]):
                                replan_flag = 1

                        if replan_flag:
                            replantime += 1 ## 排除重复规划的相同路径 0620

                        # for m in range(idx+1, next_idx+1):
                        #     print("original， The No.", m, " step: ", trajectory_plan[m])
                        #
                        # print("The length of now plan is: ", len(trajectory_optimal))
                        # for m in range(len(trajectory_optimal)):
                        #     print("trajectory_optimal， The No.", m, " step: ", trajectory_optimal[m])
                        #
                        #
                        # for m in range(len(now_trajectory)):
                        #     print("The No.", m, " step: ", now_trajectory[m])
                        # print()
                        """
                        for ll in range(idx+1):
                            temp = Point(trajectory_plan[ll].x, trajectory_plan[ll].y,
                                 trajectory_plan[ll].z, trajectory_plan[ll].ca)
                            now_trajectory.append(temp)

                        for ll in range(0, len(trajectory_optimal)):
                            temp = Point(trajectory_optimal[ll].x,trajectory_optimal[ll].y,trajectory_optimal[ll].z,trajectory_optimal[ll].ca)
                            now_trajectory.append(temp)


                        for ll in range(next_idx+1,len(trajectory_plan)):
                            temp = Point(trajectory_plan[ll].x,trajectory_plan[ll].y,trajectory_plan[ll].z,trajectory_plan[ll].ca)
                            now_trajectory.append(temp)
                        """

                        trajectory_plan = copy.deepcopy(now_trajectory)
                        # print("The UAV would move a step: ")
                        # print("The current point: ", current_p)
                        # print("The next point: ", trajectory_plan[idx+1])
                        # print("The index of next point: ", idx+1, "\n")
                # turn off camera never exist
                else:
                    # print("sensor reconfigured for the next points in the path!!!")
                    for kk in range(idx+1, next_idx+1):
                        trajectory_plan[kk].ca = 1
                sum = 0
                cam_off = 0
                for ll in range(len(trajectory_plan)):
                    sum += pri_grid_known[trajectory_plan[ll].x][trajectory_plan[ll].y][trajectory_plan[ll].z]
                    cam_off += trajectory_plan[ll].ca
                    # print("now", trajectory_plan[ll])
                # print("The length of now_trajectory_plan: ", len(trajectory_plan), sum, cam_off)

                current_f = sum + len(trajectory_plan) + cam_off

                # print("fitness", current_f)

            elif pri_grid_known[trajectory_plan[next_idx].x][trajectory_plan[next_idx].y][trajectory_plan[next_idx].z] > 0:
                trajectory_plan[next_idx].ca = 1
                # print("change sensor configuration for next point")

        time_step += 1
        idx = idx + 1
        # print("The UAV has finished this step.\n")

        #if idx == 4:
        #    print("the debug begin!")
        # print("next point", idx,time_step, len(trajectory_plan))
    # print(occ_grid)
    path_grid2 = copy.deepcopy(occ_grid)
    sum = 0
    num_ca = 0
    num_intruder = 0
    for point in trajectory_plan:
        if point.ca == 0:
            path_grid2[point.x][point.y][point.z] = 7
        else:
            path_grid2[point.x][point.y][point.z] = 10
            num_ca += 1
        sum += pri_grid[point.x][point.y][point.z]* math.exp(-(point.ca) + 1/2)
        if pri_grid[point.x][point.y][point.z] > 0:
            num_intruder += 1
        # print(point, pri_grid_known[point.x][point.y][point.z])
    print("\033[94m Fitness for replanned path:\033[0m \n", len(trajectory_plan)-1, sum, num_ca, num_intruder)

    sum = 0
    num_ca = 0
    num_intruder = 0
    for point in trajectory_ref:
        sum += pri_grid[point.x][point.y][point.z]* math.exp(-(point.ca) + 1/2)
        num_ca += point.ca
        if pri_grid[point.x][point.y][point.z] > 0:
            num_intruder += 1
        # print(point, pri_grid_known[point.x][point.y][point.z])
    print("\033[94m Fitness for reference path:\033[0m \n", len(trajectory_ref)-1, sum, num_ca, num_intruder)

    # 再次显示地图

    #print(path_grid2, sum)
    print("---------------------------------")
    print("The last plan is finished!")
    print("The length of last plan is: ", len(trajectory_plan))
    # for m in range(len(trajectory_plan)):
    #     print("The No.", m, " step: ", trajectory_plan[m])
    end = time.time()
    dtime = end - starttime
    print("程序运行时间：%.8s s" % dtime)
    #print("sumpri:", sum)
    #print("num_ca:", num_ca)
    print("\033[94m Replan times: \033[0m", replantime)
    #grid_visualization(occ_grid, starting_point, end_point, trajectory_plan, trajectory_ref)


    np.save(file="occ_grid_known.npy", arr=occ_grid_known)
    b = np.load(file="occ_grid_known.npy")
    # for m in range(grid_x):
    #     print("The value of x: ", m)
    #     print(b[m])

    plan_path = np.zeros((len(trajectory_plan),4))
    for i in range(len(trajectory_plan)):
        plan_path[i] = [trajectory_plan[i].x,trajectory_plan[i].y, trajectory_plan[i].z, trajectory_plan[i].ca]

    np.save(file="plan_path_Hybrid.npy", arr=plan_path)
    c = np.load(file="plan_path_Hybrid.npy")
    # print(c, len(c))

    exploration_rate = 0

    for i in range(grid_x):
        for j in range(grid_y):
            for k in range(grid_z):
                if occ_grid_known[i][j][k]!=occ_grid[i][j][k]:
                    exploration_rate += 1
    exploration_rate =  1 -  exploration_rate/(grid_x * grid_y * grid_z * privacy_threshold)
    print("\033[94m exploration rate: \033[0m", exploration_rate)
