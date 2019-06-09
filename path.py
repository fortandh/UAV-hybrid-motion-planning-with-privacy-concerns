# class for the path
# points: list for the point on the path
# fitness: fitness value for the path
# flag 0: feasible solution (without obstacle) 1: non feasible solution

class Path(object):
    def __init__(self, path, fitness=0, flag = 0):
        self.points = path
        self.fitness = fitness
        #self.flag = 0
