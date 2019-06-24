# class for the path
# points: list for the point on the path
# fitness: fitness value for the path
# flag 0: feasible solution (without obstacle) 1: non feasible solution


class Path(object):
    """路径类

    Attributes:
        points: 路径（点的列表）
        fitness: 适应度
        flag: 0代表路径合法（不碰障碍物）；1代表路径不合法
    """
    def __init__(self, path, fitness=0, flag=0):
        self.points = path
        self.fitness = fitness
        self.flag = flag
