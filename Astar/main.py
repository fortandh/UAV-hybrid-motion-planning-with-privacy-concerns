from Support.point import Point
from mapTools import privacy_init, map_generate
from Astar import AStar
import copy

#map parameter
grid_x = 5
grid_y = 5
grid_z = 5
grid = [grid_x, grid_y, grid_z]
safety_threshold = 0.3
privacy_threshold = 0.1
# privacy_radius = 1 ##
privacy_radius = [0.5,1,2]

# drone parameter
starting_point = Point(0, 0, 0)
end_point = Point(4, 4, 4)
T_budget = 100
viewradius = 2
Kca = 10




occ_grid, obstacle_num = map_generate(grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold)
pri_grid, sum_privacy = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

print(occ_grid)

aStar = AStar(occ_grid, pri_grid, grid, sum_privacy, starting_point, end_point, passTag = [1])
print (pri_grid)
# 开始寻路
pathList = aStar.start()
path_grid = copy.deepcopy(occ_grid)

print(len(pathList))
sum = 0
if pathList == None:
    print("no solution!")
else:
    for point in pathList:
        path_grid[point.x][point.y][point.z] = 9
        sum += pri_grid[point.x][point.y][point.z]
        print(point, pri_grid[point.x][point.y][point.z])
print("----------------------")
# 再次显示地图

print(path_grid, sum)