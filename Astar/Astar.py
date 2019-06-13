
from Point import Point
import numpy as np


class AStar:
    """
    AStar算法的Python3.x实现
    """

    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y) + abs(endPoint.z - point.z)) * 10  # 计算h值

    def __init__(self, map3d, grid, startPoint, endPoint, passTag=0):
        """
        构造AStar算法的启动条件
        :param map3d: Array2D类型的寻路数组
        :param startPoint: Point或二元组类型的寻路起点
        :param endPoint: Point或二元组类型的寻路终点
        :param passTag: int类型的可行走标记（若地图数据!=passTag即为障碍）
        """
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 寻路地图
        self.map3d = map3d
        self.grid = grid
        # 起点终点
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)

        # 障碍物标记
        self.passTag = passTag

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        :return: Node
        """
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

    def searchNear(self, minF, offsetX, offsetY, offsetZ):
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
        if self.map3d[minF.point.x + offsetX][minF.point.y + offsetY][minF.point.z + offsetZ] == self.passTag:
            return
        # 如果在关闭表中，就忽略
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY, minF.point.z + offsetZ)
        if self.pointInCloseList(currentPoint):
            return
        # 设置单位花费
        if offsetX == 0 or offsetY == 0 or offsetZ == 0:
            step = 10
        #else:
        #    step = 14
        # 如果不再openList中，就把它加入openlist
        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:
            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + step
            currentNode.father = minF

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 判断寻路终点是否是障碍
        #if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] != self.passTag:
        if self.map3d[self.endPoint.x][self.endPoint.y][self.endPoint.z] == self.passTag:
            return None

        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)
        # 2.主循环逻辑
        while True:
            # 找到F值最小的点
            minF = self.getMinNode()
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)
            # 判断这个节点的上下左右节点
            self.searchNear(minF, 0, -1, 0)
            self.searchNear(minF, 0, 1, 0)
            self.searchNear(minF, -1, 0, 0)
            self.searchNear(minF, 1, 0, 0)
            self.searchNear(minF, 0, 0, 1)
            self.searchNear(minF, 0, 0, -1)
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
    #创建一个10*10的地图
    #map3d=Array3D(10,10,10)
    map3d = np.zeros((10,10,10))
    grid = [10,10,10]
    #设置障碍
    map3d[0][4][7] = 1
    map3d[0][4][1] = 1
    map3d[0][4][2] = 1
    map3d[0][4][3] = 1
    map3d[0][4][4] = 1
    map3d[0][4][5] = 1
    map3d[0][4][6] = 1
    #显示地图当前样子
    #map3d.showArray3D()
    print(map3d)
    #创建AStar对象,并设置起点为0,0终点为9,0
    aStar=AStar(map3d, grid, Point(0,4,0),Point(0,4,9))
    #开始寻路
    pathList=aStar.start()
    #遍历路径点,在map2d上以'8'显示
    for point in pathList:
        map3d[point.x][point.y][point.z]=8
        # print(point)
    print("----------------------")
    #再次显示地图
    #map3d.showArray3D()
    print(map3d)