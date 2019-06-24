from Support.point import Point
#from Point import Point

class configure:
    def __init__(self):
        self.grid_x = 15
        self.grid_y = 15
        self.grid_z = 15
        self.grid = [self.grid_x, self.grid_y, self.grid_z]
        self.safety_threshold = 0.3
        self.privacy_threshold = 0.1
        # privacy_radius = 1 ##
        self.privacy_radius = [0.5, 1, 2]

        # drone parameter
        #self.starting_point1 = Point(0, 0, 0)
        #self.end_point1 = Point(4, 4, 4)

        self.starting_point = Point(0, 7, 0, 0)
        self.end_point = Point(14, 7, 14, 0)
        self.T_budget = 56
        self.viewradius = 2
        self.Kca = 10
