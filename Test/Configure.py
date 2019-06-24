#!/usr/bin/python
# -*- coding:utf-8 -*-

from Support.point import Point


class Configure(object):
    """配置文件

        整个系统的配置文件

        Attributes:
            grid_x, grid_y, grid_z: 地图的长宽高
            grid: 整个地图
            safety_threshold: 障碍物比例
            privacy_threshold: 隐私点比例
            starting_point: 起始点位置
            end_point: 终点位置
            T_budget: 时间硬约束
            T_optimal: 时间软约束
            view_radius: 视野半径
        """
    def __init__(self):
        self.grid_x = 15
        self.grid_y = 15
        self.grid_z = 15
        self.grid = [self.grid_x, self.grid_y, self.grid_z]
        self.safety_threshold = 0.3
        self.privacy_threshold = 0.1
        # privacy_radius = 1 ##
        self.privacy_radius = [0.5, 1, 2]

        self.starting_point = Point(0, 0, 0, 0)
        self.end_point = Point(14, 14, 14, 0)
        self.T_budget = (4/3)*(abs(self.end_point.x-self.starting_point.x) + abs(self.end_point.y-self.starting_point.y) + abs(self.end_point.z-self.starting_point.z))
        self.T_optimal = self.T_budget * 0.9
        self.view_radius = 2
