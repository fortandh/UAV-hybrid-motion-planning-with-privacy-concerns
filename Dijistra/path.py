# class for the path
# points: list for the point on the path
# sum_ca_off : times of the camera is off along the path
# collision: times of collision with obstacles:
# sum_privacy: sum of privacy risk along the path
# time_cost: time cost(path length) of the path



class Path(object):
    def __init__(self, path, length = 0, sum_ca_off = 0, collision = 0, sum_privacy = 0, time_cost = 0):
        self.points = path
        self.length = length
        self.sum_ca_off = sum_ca_off
        self.collision = collision
        self.sum_privacy = sum_privacy
        self.time_cost = time_cost
