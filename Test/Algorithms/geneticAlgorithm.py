# multi-objective genetic algorithm

import copy
import math
from random import randint, shuffle, uniform

from Support.point import Point
from Support.path import Path
from Support.commonPoint import CommonPoint
from Support.quickSort import quick_sort


class GeneticAlgorithm(object):
    """算法类——遗传算法

    Attributes:
        population: 种群中个体的数量
    """
    def __init__(self, population):
        # population 种群数量
        self.population = population

    def init_population(self, starting_point, objectives, Kca, grid_x, grid_y, grid_z):
        """初始化种群

        Args:
            starting_point: 起始点的坐标
            objectives: 目标列表
            Kca: 隐私点等级
            grid_x，grid_y，grid_z: 三维空间长宽高

        Returns:
            max_f: 最优路径的fitness
            max_path: 最优路径
            max_flag: 最优路径的flag
        """
        solutions = []
        for i in range(self.population):
            objectives_ = objectives[:]  # copy
            prev_x = starting_point.x
            prev_y = starting_point.y
            prev_z = starting_point.z
            prev_ca = starting_point.ca
            total_obj = len(objectives_)
            tmp = [starting_point]
            shuffle(objectives_)
            num_of_ca_off = 0
            # 生成一个可行解,这个解要求通过所有的点，不论什么顺序
            while total_obj > 0:
                next_x = 0
                next_y = 0
                next_z = 0
                next_ca = 0

                # range of x
                # min_x = prev_x
                # max_x = prev_x
                # if objectives_[0].x < prev_x:
                #     min_x = prev_x - 1
                # if objectives_[0].x > prev_x:
                #     max_x = prev_x + 1
                min_x = 0
                if prev_x - 1 > 0:
                    min_x = prev_x - 1
                max_x = grid_x - 1
                if prev_x + 1 < grid_x:
                    max_x = prev_x + 1

                # range of y
                # min_y = prev_y
                # max_y = prev_y
                # if objectives_[0].y < prev_y:
                #     min_y = prev_y - 1
                # if objectives_[0].y > prev_y:
                #     max_y = prev_y + 1
                min_y = 0
                if prev_y - 1 > 0:
                    min_y = prev_y - 1
                max_y = grid_y - 1
                if prev_y + 1 < grid_y:
                    max_y = prev_y + 1

                # range of z
                # min_z = prev_z
                # max_z = prev_z
                # if objectives_[0].z < prev_z:
                #     min_z = prev_z - 1
                # if objectives_[0].z > prev_z:
                #     max_z = prev_z + 1
                min_z = 0
                if prev_z - 1 > 0:
                    min_z = prev_z - 1
                max_z = grid_z - 1
                if prev_z + 1 < grid_z:
                    max_z = prev_z + 1

                # 这里available表示x,y,z可取的值，max加1表示末尾算入available中
                available_x = range(min_x, max_x + 1)
                available_y = range(min_y, max_y + 1)
                available_z = range(min_z, max_z + 1)

                # generation of the next point
                allowed_point = True
                while allowed_point:
                    next_x = available_x[randint(0, len(available_x) - 1)]
                    next_y = available_y[randint(0, len(available_y) - 1)]
                    next_z = available_z[randint(0, len(available_z) - 1)]

                    if (abs(prev_x - next_x) + abs(prev_y - next_y) + abs(prev_z - next_z)) == 1:
                        allowed_point = False

                # the next option for camera configuration
                rand_ca = randint(0, 1)
                if rand_ca == 1:
                    if num_of_ca_off + 1 <= Kca:
                        num_of_ca_off += 1
                        next_ca = rand_ca
                    else:
                        next_ca = 0
                else:
                    next_ca = rand_ca

                # print(next_ca, num_of_ca_off, Kca)
                # add the point to the ath
                prev_x = next_x
                prev_y = next_y
                prev_z = next_z
                prev_ca = next_ca
                tmp.append(Point(next_x, next_y, next_z, next_ca))
                for o in range(total_obj):
                    if next_x == objectives_[o].x and next_y == objectives_[o].y and next_z == objectives_[o].z:
                        total_obj -= 1
                        del objectives_[o]
                        if total_obj > 0:
                            shuffle(objectives_)
                        break
            solutions.append(Path(tmp))
        return solutions

    '''
    # 交叉过程
    def cross_over(self, path1, path2, objectives):
        # 寻找公共点
        tmp = list(set(path1).intersection(path2))
        common_points = []
        for c in tmp:
            common_points.append(CommonPoint(c, path1.index(c), path2.index(c)))
        common_points.sort(key=lambda x: x.path2_index)

        common_points_ = common_points[:]

        i = 1
        # Eliminate consecutive common points
        # i=0时为起点，因此不能计算在内
        # 消除连续的共同点
        while i + 1 < len(common_points):
            for j in range(i, len(common_points)):
                if common_points[j].path2_index - common_points[i].path2_index <= 1:
                    if j + 1 < len(common_points):
                        # Keep the last point of the consecutive points
                        if common_points[j + 1].path2_index - common_points[j].path2_index == 1:
                            if common_points[j].point not in objectives:
                                common_points_.remove(common_points[j])
                    i = j
                else:
                    i += 1
                    break

        common_points_1 = common_points_[:]
        common_points_1.sort(key=lambda x: x.path1_index)

        c_tmp = common_points_[:]
        c1_tmp = common_points_1[:]

        # Eliminate points that are not in the same order (thus, not part of the same sub-path)
        for tc in range(1, len(c_tmp)):
            keep_it = False
            tc1 = c1_tmp.index(c_tmp[tc])
            # 两段的开头和结尾相同
            if c_tmp[tc - 1] == c1_tmp[tc1 - 1]:
                if tc + 1 < len(c_tmp) and tc1 + 1 < len(c1_tmp):
                    if c_tmp[tc + 1] == c1_tmp[tc1 + 1] and c_tmp[tc - 1] in common_points_:
                        keep_it = True
            if not keep_it:
                del common_points_[common_points_.index(c_tmp[tc])]
                del common_points_1[common_points_1.index(c1_tmp[tc1])]

        # 选择交叉点——起点
        crossover_point = 0
        if len(common_points_) > 1:
            crossover_point = randint(0, len(common_points_) - 1)
            while common_points_[crossover_point] == common_points_1[-1]:
                crossover_point = randint(0, len(common_points_) - 2)

        # 选择交叉点——终点
        crossover_stop = common_points_1.index(common_points_[crossover_point]) + 1

        # 第二段路径中的头半段
        first_half = copy.deepcopy(path2[:common_points_[crossover_point].path2_index])
        new_part = []
        # 中间的交换部分，从第一条路径中提取
        if crossover_stop < len(common_points_1):
            new_part = copy.deepcopy(
                path1[common_points_[crossover_point].path1_index:common_points_1[crossover_stop].path1_index + 1])
        else:
            new_part = copy.deepcopy(path1[common_points_[crossover_point].path1_index:])
        second_half = []
        # 第二段路径中的后半段
        if crossover_stop < len(common_points_1):
            if not common_points_1[crossover_stop].point == path1[-1]:
                second_half = copy.deepcopy(path2[common_points_1[crossover_stop].path2_index + 1:])
        new_path = first_half + new_part + second_half

        return new_path
    '''

    def cross_over(self, path1, path2, objectivesm, Kca):
        """交叉过程

        将path2的头部分和path1的尾部分结合起来。

        Args:
            path1,path2: 路径
            objectives: 目标列表
            Kca: 关闭摄像头的次数

        Returns:
            Path(new_path): 一条新的路径规划
        """
        # 寻找公共点
        tmp = list(set(path1).intersection(path2))
        common_points = []

        for c in tmp:
            common_points.append(CommonPoint(c, path1.index(c), path2.index(c)))
        common_points.sort(key=lambda x: x.path2_index)

        # 选定中间交叉点
        if len(common_points) > 2:
            crossover_point = randint(1, len(common_points) - 2)
        else:
            new_path = copy.deepcopy(path2[:])
            return Path(new_path)

        num_of_ca_off1 = 0
        num_of_ca_off2 = 0

        for i in range(common_points[crossover_point].path2_index+1):
            if path2[i].ca == 1:
                num_of_ca_off2 += 1
        for i in range(common_points[crossover_point].path1_index+1, len(path1)):
            if path1[i].ca == 1:
                num_of_ca_off1 += 1

        # 开始进行交叉
        first_half = copy.deepcopy(path2[:common_points[crossover_point].path2_index])
        new_part = copy.deepcopy(path1[common_points[crossover_point].path1_index:])
        # new_path = first_half + new_part

        if (num_of_ca_off2 + num_of_ca_off1) <= Kca:
            new_path = first_half + new_part
            # return Path(new_path)
        else:
            left = num_of_ca_off1 - (Kca - num_of_ca_off2)
            for j in range(len(new_part)):
                if new_part[j].ca == 1:
                    new_part[j].ca = 0
                    left -= 1
                if left == 0:
                    break
            new_path = first_half + new_part

        return Path(new_path)

    def mutate(self, path, objectives, Kca, grid_x, grid_y, grid_z):
        """变异过程

        从path中某一点开始，重新延申出一条新的规划。

        Args:
            path: 路径
            objectives: 目标列表
            Kca: 关摄像头的次数
            grid_x,grid_y,grid_z: 3维空间的长宽高

        Returns:
            new_path: 一条新的路径规划
        """
        # choose the starting point for mutation
        if len(path.points) < 3:
            return path
        starting_point = randint(1, len(path.points) - 2)

        objectives_ = objectives[:]
        prev_x = path.points[starting_point].x
        prev_y = path.points[starting_point].y
        prev_z = path.points[starting_point].z
        prev_ca = path.points[starting_point].ca
        num_of_ca_off = 0

        for i in range(starting_point+1):
            if path.points[i].ca == 1:
                num_of_ca_off += 1

        total_obj = len(objectives_)
        # tmp = [path.points[starting_point]]
        tmp = copy.deepcopy(path.points[:starting_point+1])
        # sort the objectives randomly
        # shuffle(objectives_)
        while total_obj > 0:
            allowed_point = True
            next_x = 0
            next_y = 0
            next_z = 0
            next_ca = 0

            # range of x
            min_x = 0
            if prev_x - 1 > 0:
                min_x = prev_x - 1
            max_x = grid_x - 1
            if prev_x + 1 < grid_x:
                max_x = prev_x + 1

            # range of y
            min_y = 0
            if prev_y - 1 > 0:
                min_y = prev_y - 1
            max_y = grid_y - 1
            if prev_y + 1 < grid_y:
                max_y = prev_y + 1

            # range of z
            min_z = 0
            if prev_z - 1 > 0:
                min_z = prev_z - 1
            max_z = grid_z - 1
            if prev_z + 1 < grid_x:
                max_z = prev_z + 1

            available_x = range(min_x, max_x + 1)
            available_y = range(min_y, max_y + 1)
            available_z = range(min_z, max_z + 1)

            while allowed_point:

                next_x = available_x[randint(0, len(available_x) - 1)]
                next_y = available_y[randint(0, len(available_y) - 1)]
                next_z = available_z[randint(0, len(available_z) - 1)]

                if (abs(prev_x - next_x) + abs(prev_y - next_y) + abs(prev_z - next_z)) == 1:
                    allowed_point = False

                if prev_x == next_x and prev_y == next_y and prev_z == next_z:
                    allowed_point = True

            # the next option for camera configuration
            rand_ca = randint(0, 1)
            if rand_ca == 1:
                if num_of_ca_off + 1 <= Kca:
                    num_of_ca_off += 1
                    next_ca = rand_ca
                else:
                    next_ca = 0
            else:
                next_ca = rand_ca

            prev_x = next_x
            prev_y = next_y
            prev_z = next_z

            tmp.append(Point(next_x, next_y, next_z, next_ca))
            # print(next_x,next_y,next_z)
            # print(Point)
            for o in range(total_obj):
                if next_x == objectives_[o].x and next_y == objectives_[o].y and next_z == objectives_[o].z:
                    total_obj -= 1
                    del objectives_[o]
                    if total_obj > 0:
                        shuffle(objectives_)
                    break
        # first_half = copy.deepcopy(path.points[:starting_point])
        # next_half = tmp
        # new_path = Path(first_half + next_half)

        new_path = Path(tmp)
        return new_path

    def smooth(self, path):
        """平滑过程

        删除规划中头尾相同的片段，防止路径规划中出现环，缩短路径长度。

        Args:
            path: 路径
        Returns:
            Path(old_points): 一条经过简化的路径
        """
        old_points = copy.deepcopy(path.points)
        changed = True
        while changed:
            changed = False
            a = 0
            b = 0
            # print("The length of old_points: ", len(old_points))
            for i in range(0, len(old_points)):
                # print("The value of i in smooth: ", i)
                if old_points[i] in old_points[i+1:]:
                    a = i
                    b = old_points[i+1:].index(old_points[i])+i+1
                    changed = True
                    break
            if changed:
                new_points = old_points[:a]
                new_points.extend(old_points[b:])
                old_points = new_points
        return Path(old_points)

    # def tournament_select(self, paths):
    #     winner_index = 0
    #     best_fitness = -9999999
    #
    #     for i in range(self.tournament_size):
    #         rand_index = randint(0, len(paths) - 1)
    #         if paths[rand_index].fitness > best_fitness:
    #             best_fitness = paths[rand_index].fitness
    #             winner_index = rand_index
    #
    #     return winner_index

    def select(self, paths, selection_size, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num):
        """选择方法

        使用了随机抽样的方法，并且使用指数函数去除负值，使用轮盘赌的方式进行选择。

        Args:
            paths: 路径列表
            selection_size: 选择规模
            occ_grid: 整个地图
            pri_grid: 隐私点影响分布图
            start: 起始点坐标
            end: 终点坐标
            sum_privacy: 隐私影响总和
            obstacle_num: 障碍物总数

        Returns:
            selected_path_list: 被选中的路径的列表
        """
        # 计算fitness之和
        fitness_sum = 0
        for path in paths:
            fitness_sum += math.exp(self.get_fitness(path, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num))

        # 构建轮盘
        tmp = 0
        roulette_list = []
        for path in paths:
            fitness = math.exp(self.get_fitness(path, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num))
            tmp = tmp + (fitness / fitness_sum)
            roulette_list.append(tmp)

        # 开始选择
        interval = 1.0 / selection_size
        selection = uniform(0, interval)
        selected_path_list = []
        for i in range(selection_size):
            choice = selection + i * interval
            selected_id = 0
            for j in range(len(roulette_list)):
                if choice <= roulette_list[j]:
                    selected_id = j
                    break
            selected_path_list.append(paths[selected_id])
        return selected_path_list

    # 重插入方法
    # old_paths 父代路径
    # new_paths 子代路径
    # population 种群规模
    def reinsert(self, old_paths, new_paths, population):
        """重插入过程

        将父代与子代融合，选出其中较优的个体。

        Args:
            old_paths: 父代个体集合
            new_paths: 子代个体集合
            population: 种群中个体的数量的上限
        Returns:
            tmp_paths[:population]: 适应度较好的个体的集合
        """
        # 父子融合
        tmp_paths = old_paths
        tmp_paths.extend(new_paths)
        # 优胜劣汰
        quick_sort(tmp_paths)
        return tmp_paths[:population]

    def get_fitness(self, path, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num):
        """计算一个路径规划的适应度

        Args:
            path: 路径列表
            occ_grid: 整个地图
            pri_grid: 隐私点影响分布图
            start: 起始点坐标
            end: 终点坐标
            sum_privacy: 隐私影响总和
            obstacle_num: 障碍物总数

        Returns:
            fitness: 一条路径规划的适应度值
        """
        # 系数设定
        safety = 0
        privacy = 0
        flag = 0
        alpha = 1
        beta = 1
        num_cam_off = 0.1

        for point in path.points:
            # The closer to an obstacle, the more points lost
            if occ_grid[point.x][point.y][point.z] == 1:
                flag += -2
                safety += occ_grid[point.x][point.y][point.z]
            if point.ca == 1:
                num_cam_off += 1
        # print (safety)
        for point in path.points:
            privacy += math.exp(point.ca) * pri_grid[point.x][point.y][point.z]
        # h*exp(-ws-(1/2)*dis^2)
        # print(privacy)
        if sum_privacy == 0:
            privacy = 0
        else:
            privacy = 1 - privacy/sum_privacy

        ideal_length = abs(start.x - end.x) + abs(start.y - end.y) + abs(start.z - end.z)
        length = len(path.points)
        efficiency = ideal_length / (length - 1)
        # safety = 1 - safety / obstacle_num  # 可以统计所有的障碍物的数目
        # print(ideal_length,length,efficiency)
        # print(obstacle_num, safety)
        # print(sum_privacy,privacy)
        # print(flag)
        # maximize
        # fitness = flag + beta * efficiency + alpha * privacy
        fitness = flag + beta * efficiency + alpha * privacy + 1/num_cam_off
        # fitness = beta * efficiency + alpha * privacy
        # print(fitness)
        if flag < 0:
            path.flag = 1
        # print (flag, path.flag)
        return fitness
