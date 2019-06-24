# the common points on two path, used for crossover


class CommonPoint(object):
    """公共点类

    描述两条路径规划中的公共点，用于交叉过程。

    Attributes:
        point: 公共点
        path1_index: 公共点在path1中的编号（从0开始）
        path2_index: 公共点在path2中的编号（从0开始）
    """
    def __init__(self, point, path1_index=0, path2_index=0):
        self.point = point
        self.path1_index = path1_index
        self.path2_index = path2_index

    def __str__(self):
        return str(self.point) + " | " + str(self.path1_index) + " | " + str(self.path2_index)

    def __eq__(self, p):
        return self.point == p.point and self.path1_index == p.path1_index and self.path2_index == p.path2_index
