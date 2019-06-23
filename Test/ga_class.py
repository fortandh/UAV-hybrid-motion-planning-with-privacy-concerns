#!/usr/bin/env python
# main function

from point import Point
from GA import geneticAlgorithm as gA
from GA.quickSort import quick_sort
from GA.mapTools import privacy_init, map_generate
import copy
import sys
sys.setrecursionlimit(1000000)

path = []
grid_x = 5
grid_y = 5
grid_z = 5

occ_grid = None
population = 500
generation = 50
selection_size = 50

objectives = []
no_of_objectives = 1
obstacles_per_axis = 1
starting_point = Point(0, 0, 0, 0)
end_point = Point(4, 4, 4, 0)


safety_threshold = 0.5
privacy_threshold = 0.1
privacy_radius = [0.5,1,2]
# the tolerance of the times of camera off
Kca = 8

Tbudget = 100


class GA_class(object):
    """算法类——遗传算法

    这个类实现了遗传算法，主要用来初始规划的生成。

    Attributes:
        population: 种群中个体的数量
        generation: 种群进化的代数
        selection_size: 选择过程中选入个体的数量
    """
    def __init__(self, population, generation, selection_size):
        self.population = population
        self.generation = generation
        self.selection_size = selection_size

    # 初始化种群
    def initialsolution(self, occ_grid, pri_grid, privacy_sum, obstacle_num, starting_point, end_point, Tbudget, Kca, grid_x, grid_y, grid_z):
        """生成初始规划

        使用遗传算法来完成初始解的生成。

        Args:
            occ_grid: 全局地图
            pri_gird: 隐私点影响图
            privacy_sum: 隐私点的数目
            obstacle_num: 障碍物的数目
            starting_point: 起始点的坐标
            end_point: 终点的坐标
            Tbudget: 路径规划的最长长度
            Kca: 隐私点等级
            grid_x: 地图中X的最大数值
            grid_y: 地图中Y的最大数值
            grid_z: 地图中Z的最大数值

        Returns:
            max_f: 最优路径的fitness
            max_path: 最优路径
            max_flag: 最优路径的flag

        """
        # 初始化变量
        objectives = [end_point]
        Kca = Kca
        T_budget = Tbudget
        occ_grid = occ_grid
        pri_grid = pri_grid
        privacy_sum = privacy_sum
        obstacle_num = obstacle_num
        starting_point = starting_point
        end_point = end_point

        alg = gA.GeneticAlgorithm(self.population)
        print('\033[94m Generating random initial solutions for global planning... \033[0m')
        paths = alg.init_population(starting_point, objectives, Kca, grid_x, grid_y, grid_z)
        print('\033[94m Solutions generated.\033[0m')
        for i in range(len(paths)):
            # print("The value of i in ga_class: ", i)
            paths[i] = alg.smooth(paths[i])
        print('\033[94m Smooth method finished.\033[0m')

        for p in range(len(paths)):
            paths[p].fitness = alg.get_fitness(paths[p], occ_grid, pri_grid,
                                               starting_point, end_point, privacy_sum, obstacle_num)

        max_p = max(paths, key=lambda x: x.fitness)
        # print(max_p)

        max_f = -5
        max_path = copy.deepcopy(paths[0])
        delete_idx = []
        max_flag = 1

        for i in range(self.generation):
            print("The generation num: ", i)
            quick_sort(paths)

            for m in range(len(paths)):
                # print (m)
                if max_f <= paths[m].fitness and paths[m].flag == 0:
                    max_f = paths[m].fitness
                    max_path = copy.deepcopy(paths[m])
                    max_flag = paths[m].flag
                    print('\033[94m Current maximum fitness:\033[0m\033[92m ' + str(
                        max_f) + '\033[0m\033[94m, Generation:\033[0m\033[92m ' + str(i) + '\033[0m\033[94m, Flag:\033[0m\033[92m ' +  str(max_flag) +
                          '\033[0m\033[94m, Solution:\033[0m\033[92m ' +str(m) +' \033[0m')
                    for j in range(len(paths[m].points)):
                        print(paths[m].points[j])
                    print("the solution", m, len(paths[m].points), max_flag)
                    break
                    # alg.get_fitness(paths[0], occ_grid, pri_grid, starting_point, end_point, privacy_sum, obstacle_num)
            # 选择
            selected_path_list = alg.select(paths, self.selection_size, occ_grid, pri_grid, starting_point,
                                            end_point, privacy_sum, obstacle_num)
            # 形成couple, 交叉， 变异
            new_path_list = []

            for j in range(self.selection_size):
                for k in range(j + 1, self.selection_size):
                    new_path1 = alg.cross_over(selected_path_list[j].points, selected_path_list[k].points, objectives, Kca)
                    new_path2 = alg.cross_over(selected_path_list[k].points, selected_path_list[j].points, objectives, Kca)

                    new_path1_mutated = alg.mutate(new_path1, objectives, Kca, grid_x, grid_y, grid_z)
                    new_path1_mutated.fitness = alg.get_fitness(new_path1_mutated, occ_grid, pri_grid,
                                                                starting_point, end_point, privacy_sum,
                                                                obstacle_num)
                    new_path_list.append(new_path1_mutated)

                    new_path2_mutated = alg.mutate(new_path2, objectives, Kca, grid_x, grid_y, grid_z)
                    new_path2_mutated.fitness = alg.get_fitness(new_path2_mutated, occ_grid, pri_grid,
                                                                starting_point, end_point, privacy_sum,
                                                                obstacle_num)
                    new_path_list.append(new_path2_mutated)
            # 重插入
            paths = alg.reinsert(paths, new_path_list, population)

        if len(max_path.points) < T_budget:
            return max_f, max_path, max_flag
        else:
            print("No Solution!")
            return max_f, max_path, max_flag


# import global map
def initialmap (grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold, privacy_radius):
    #print("start")
    occ_grid, obstacle_num = map_generate(grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold)
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)

    occ_grid_known = copy.deepcopy(occ_grid)

    for i in range (grid_x):
        for j in range (grid_y):
            for k in range (grid_z):
                if occ_grid[i][j][k] == 2 or occ_grid[i][j][k] == 3 or occ_grid[i][j][k] == 4:
                    occ_grid_known[i][j][k] = 0
                    # print (occ_grid_known[i][j][k], i,j,k)
    pri_grid_known, privacy_sum_known = privacy_init(grid_x, grid_y, grid_z, occ_grid_known, privacy_radius)
    # print (occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known)
    return occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known


if __name__ == "__main__":
    occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known = initialmap(grid_x, grid_y, grid_z, starting_point, end_point, safety_threshold, privacy_threshold, privacy_radius)

    print(occ_grid, obstacle_num, occ_grid_known, pri_grid_known, privacy_sum_known)

    ga = GA_class(population, generation, selection_size)

    max_f, max_path, max_flag = ga.initialsolution(occ_grid, pri_grid_known, privacy_sum_known, obstacle_num, starting_point, end_point, Tbudget, Kca, grid_x, grid_y, grid_z)
    print(max_f, max_flag)

    for j in range(len(max_path.points)):
        print(max_path.points[j])


