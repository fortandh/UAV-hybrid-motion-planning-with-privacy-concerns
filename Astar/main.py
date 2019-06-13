from Point import Point
import numpy as np
from mapTools import privacy_init, map_generate
from Astar import AStar


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

print(occ_grid)
# 创建AStar对象,并设置起点为0,0终点为9,0
aStar = AStar(occ_grid, grid, starting_point, end_point, passTag = 1)
# 开始寻路
pathList = aStar.start()
# 遍历路径点,在map2d上以'8'显示\
print(len(pathList))
for point in pathList:
    occ_grid[point.x][point.y][point.z] = 9
    # print(point)
print("----------------------")
# 再次显示地图
# map3d.showArray3D()
print(occ_grid)