#!/usr/bin/python
# -*- coding:utf-8 -*-


class Point(object):
    """表示一个点

    Attributes:
        x, y, z: 坐标
        ca: camera的状态，0代表开，1代表关
    """

    def __init__(self, x, y, z, ca):
        self.x = x
        self.y = y
        self.z = z
        self.ca = ca

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.z == other.z and self.ca == other.ca:
            return True
        return False

    def __str__(self):
        return "x:" + str(self.x) + ",y:" + str(self.y) + ",z:" + str(self.z) + ",ca:" + str(self.ca)

    def __hash__(self):
        return hash(str(self))
