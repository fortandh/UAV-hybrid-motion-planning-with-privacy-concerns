from Point2 import Point
#from Point import Point

class configure:
    def __init__(self):
        self.grid_x = 5
        self.grid_y = 5
        self.grid_z = 5
        self.grid = [self.grid_x, self.grid_y, self.grid_z]
        self.safety_threshold = 0.3
        self.privacy_threshold = 0.2
        # privacy_radius = 1 ##
        self.privacy_radius = [0.5, 1, 2]

        # drone parameter
        #self.starting_point = Point(0, 0, 0)
        #self.end_point = Point(4, 4, 4)
        self.starting_point = Point(0, 0, 0, 0)
        self.end_point = Point(4, 4, 4, 0)
        self.T_budget = 20
        self.viewradius = 2
        self.Kca = 10
