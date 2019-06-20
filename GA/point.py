# class for the state on the path
# ca:camera's working state 1: off 0: on

class Point(object):
    def __init__(self, x, y, z, ca=0):
        self.x = x
        self.y = y
        self.z = z
        self.ca = ca

    def __str__(self):
        return "[" + str(self.x) + "," + str(self.y) + "," + str(self.z) + "," + str(self.ca) + "]"

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y and self.z == p.z and self.ca == p.ca

    def __hash__(self):
        return hash(str(self))
