"""
add camera into searching space
"""

from Point2 import Point
import numpy as np
from mapTools import privacy_init, map_generate, hasprivacythreat, initialmap, hasprivacythreat2
import copy
from Configure import configure
import math
import random


class AStar:
    """
    AStar算法的Python3.x实现
    """

    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, ideallength, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.step = 1
            self.cam = 0
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y) + abs(endPoint.z - point.z))/ideallength # 计算h值 曼哈顿距离

        def __str__(self):
            return "x:" + str(self.point.x) + ",y:" + str(self.point.y) + ",z:" + str(self.point.z)

    def __init__(self, occ_grid, pri_grid, grid, sum_privacy, startPoint, endPoint, passTag, Tbudget, threat_list):
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
        self.threatlist = threat_list
        self.timestep = 0
        #self.startPoint = startPoint
        #self.endPoint = endPoint

        # 起点终点
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)

        # 障碍物标记
        self.passTag = passTag

    """new function"""

    def updateNodeHvalue(self):
        for i in range(len(self.openList)):
            node = self.openList[i]
            #print("#######",node)
            #print("$$$", node.point.x)
            rou1 = (abs(node.point.x - self.startPoint.x) + abs(node.point.y - self.startPoint.y) + abs(
                node.point.z - self.startPoint.z)) / self.ideallength
            rou2 = node.step / self.Tbudget
            adaptive1 = math.exp(1 - rou1 / rou2)
            adaptive2 = math.exp(rou1 / rou2 - 1)
            #print(rou1,rou2, adaptive1, adaptive2)
            fathernode = node.father
            #print("*******", node.point.x, fathernode.point.x)
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
                    delta_h += adaptive1 * self.map3d[threat[0]][threat[1]][threat[2]]
                else:
                    delta_h += adaptive2 * self.map3d[threat[0]][threat[1]][threat[2]]
            node.h = node.h * delta_h
            #print("node.h:", node.h)

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        :return: Node
        """
        self.updateNodeHvalue()
        currentNode = self.openList[0]

        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode



    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
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
        if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.grid[0] - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.grid[1] - 1 or minF.point.z + offsetZ < 0 or \
                minF.point.z + offsetZ > self.grid[2] - 1:
            return
        # 如果是障碍，就忽略
        #if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] != self.passTag:
        #if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] in self.passTag:

        if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] in self.passTag:
            return
        # 如果在关闭表中，就忽略
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY, minF.point.z + offsetZ, cam)
        if self.pointInCloseList(currentPoint):
            return
        # 设置单位花费

        step = 1/self.ideallength

        if self.sumpri == 0:
            privacy_threat = 0
        else:
            privacy_threat = (self.prigrid[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] * math.exp(-(cam)))
        cam_off = cam

       # delta_g = step + privacy_threat
        delta_g = step + cam_off + privacy_threat

        # 如果不再openList中，就把它加入openlist
        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:
            currentNode = AStar.Node(currentPoint, self.endPoint, self.ideallength, g=minF.g + delta_g)
            currentNode.father = minF
            currentNode.cam = minF.cam + cam
            currentNode.step = minF.step + 1
            self.openList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        if minF.g + delta_g < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + delta_g
            currentNode.father = minF
            currentNode.step = minF.step + 1
            currentNode.cam = minF.cam

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 判断寻路终点是否是障碍
        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] != self.passTag:
        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] in self.passTag:
        if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] in self.passTag:
            return None
        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint, self.ideallength)
        self.openList.append(startNode)
        # 2.主循环逻辑
        while True:
            # 找到F值最小的点
            minF = self.getMinNode()
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)

            # 判断这个节点的上下左右节点
            # turn on camera
            #actions = [[0, -1, 0, 0],[0, 1, 0, 0],[-1, 0, 0, 0],[1, 0, 0, 0],[0, 0, 1, 0],[0, 0, -1, 0],[0, -1, 0, 1],[0, 1, 0, 1],[-1, 0, 0, 1],[0, 0, 1, 1],[0, 0, -1, 1],[1, 0, 0, 1]]
            #actionlist = [0,1,2,3,4,5,6,7,8,9,10,11]
            #random.shuffle(actionlist)

            #for i in range (len(actionlist)):
            #    self.searchNear(minF, actions[actionlist[i]][0], actions[actionlist[i]][1], actions[actionlist[i]][2], actions[actionlist[i]][3])
            #"""
            # turn on camera
            self.searchNear(minF, 0, -1, 0, 0)
            self.searchNear(minF, 0, 1, 0, 0)
            self.searchNear(minF, -1, 0, 0, 0)
            self.searchNear(minF, 1, 0, 0, 0)
            self.searchNear(minF, 0, 0, 1, 0)
            self.searchNear(minF, 0, 0, -1, 0)
            # turn off camera
            self.searchNear(minF, 0, -1, 0, 1)
            self.searchNear(minF, 0, 1, 0, 1)
            self.searchNear(minF, -1, 0, 0, 1)
            self.searchNear(minF, 1, 0, 0, 1)
            self.searchNear(minF, 0, 0, 1, 1)
            self.searchNear(minF, 0, 0, -1, 1)
            #"""

            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                # print("关闭表中")
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
    viewradius = config.viewradius
    Kca = config.Kca
    threat_list = []

    occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap(grid_x, grid_y, grid_z,
                                                                                           starting_point, end_point,
                                                                                           safety_threshold,
                                                                                           privacy_threshold,
                                                                                           privacy_radius)

    aStar = AStar(occ_grid, pri_grid_known, grid, privacy_sum_known, starting_point, end_point, [1], T_budget, threat_list)


    # 开始寻路
    trajectory_ref = aStar.start()
    path_grid = copy.deepcopy(occ_grid)

    #print(len(pathList))
    sum = 0
    if trajectory_ref == None:
        print("no solution!")
    else:
        for point in trajectory_ref:
            path_grid[point.x][point.y][point.z] = 9
            sum += pri_grid_known[point.x][point.y][point.z]
           # print(point, pri_grid_known[point.x][point.y][point.z])
    print("----------------------", len(trajectory_ref))
    # 再次显示地图

    print(path_grid, sum)

    trajectory_plan = copy.deepcopy(trajectory_ref)
    sensor_initial = np.zeros(len(trajectory_plan))
    sensor_plan = copy.deepcopy(sensor_initial)
    time_step = 0

    idx = 0
    current_f = sum + len(trajectory_plan)


    while not (idx >= len(trajectory_plan)):
        current_p = trajectory_plan[idx]
        current_ca = trajectory_plan[idx].ca

        if current_p == end_point :
            break

        next_p = trajectory_plan[idx+1]
        next_idx = idx + 1
        print (current_p,next_p,next_idx)

        if current_ca == 1:
            time_step += 1
            current_p = next_p
            idx += 1
            continue

        # take picture
        # update occ_grid, pri_grid
        flag, occ_grid_known, pri_grid_known, privacy_sum_known, threat_list = hasprivacythreat2 (current_p, occ_grid_known, occ_grid, pri_grid_known, privacy_sum_known, viewradius)

        if flag:
            # localization
            # p_threat, h_impact = privacy_modeling()
            # update occ_grid, pri_grid

            for j in range (idx+1, len(trajectory_plan)):
                sigma_privacy = 0
                for k in range (j,len(trajectory_plan)):
                    sigma_privacy += pri_grid_known[trajectory_plan[k].x][trajectory_plan[k].y][trajectory_plan[k].z]* math.exp(-(trajectory_plan[k].ca))
                if sigma_privacy == 0:
                    next_p = trajectory_plan[j]
                    next_idx = j
                    break
                elif k == len(trajectory_plan)-1 :
                    next_p = trajectory_plan[-1]
                    next_idx = len(trajectory_plan)-1
            print(next_idx,next_p)
            if next_idx != idx + 1: # no need for motion planning

                T_plan = T_budget - len(trajectory_plan) + (next_idx - idx)
                #if T_plan < (abs(trajectory_plan.points[next_idx].x - trajectory_plan.points[idx].x) + abs(trajectory_plan.points[next_idx].y - trajectory_plan.points[idx].y) + \
                #        abs(trajectory_plan.points[next_idx].z - trajectory_plan.points[idx].z)):
                #    print("no solution!")
                print(T_plan, current_p,  next_p)
                aStar = AStar(occ_grid, pri_grid_known, grid, privacy_sum_known, current_p, next_p, [1], T_budget, threat_list)

                # 开始寻路
                trajectory_optimal = aStar.start()

                previous_trajectory = copy.deepcopy(trajectory_plan[ :idx])
                following_trajectory = copy.deepcopy(trajectory_plan[next_idx+1: ])

                now_trajectory = []
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



                trajectory_plan = copy.deepcopy(now_trajectory)

                sum = 0
                cam_off = 0
                for ll in range(len(trajectory_plan)):
                    sum += pri_grid_known[trajectory_plan[ll].x][trajectory_plan[ll].y][trajectory_plan[ll].z]
                    cam_off += trajectory_plan[ll].ca
                    print("now", trajectory_plan[ll])
                #print("The length of now_trajectory_plan: ", len(trajectory_plan), sum, cam_off)

                current_f = sum + len(trajectory_plan) + cam_off

                #print("fitness", current_f)
        time_step += 1
        idx = idx + 1
        print(idx,time_step, len(trajectory_plan))