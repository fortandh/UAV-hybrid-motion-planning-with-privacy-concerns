# multi-objective genetic algorithm
import copy
from random import randint, shuffle, uniform
from point import Point
from path import Path
from numpy import random
from commonPoint import CommonPoint
from quickSort import quick_sort
import math


class GeneticAlgorithm(object):
    """
    def __init__(self, population, sigma, mr, ts, sr, grid_map):
        # population 种群数量
        self.population = population
        # std 标准差，进行变异时正态分布的参数
        self.std = sigma
        # mutation_rate 变异率
        self.mutation_rate = mr
        # tournament_size 进行锦标赛的个体的数量
        self.tournament_size = ts
        # smoothing_rate 进行平滑过程的概率
        self.smoothing_rate = sr
        self.scale = grid_map

    """
    def __init__(self, population):
        # population 种群数量
        self.population = population


    # 初始化种群
    def init_population(self, starting_point, objectives, Kca):
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
                # range of x
                min_x = prev_x
                max_x = prev_x
                if objectives_[0].x < prev_x:
                    min_x = prev_x - 1
                if objectives_[0].x > prev_x:
                    max_x = prev_x + 1

                # range of y
                min_y = prev_y
                max_y = prev_y
                if objectives_[0].y < prev_y:
                    min_y = prev_y - 1
                if objectives_[0].y > prev_y:
                    max_y = prev_y + 1

                # range of z
                min_z = prev_z
                max_z = prev_z
                if objectives_[0].z < prev_z:
                    min_z = prev_z - 1
                if objectives_[0].z > prev_z:
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
                #print(next_ca, num_of_ca_off, Kca)
                # add the point to the path
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
        #new_path = first_half + new_part

        if (num_of_ca_off2 + num_of_ca_off1) <= Kca :
            new_path = first_half + new_part
            #return Path(new_path)
        else:
            left = num_of_ca_off1 - (Kca - num_of_ca_off2)
            for j in range(len(new_part)):
                if new_part[j].ca == 1:
                    new_part[j].ca = 0
                    left -= 1
                if left == 0:
                    break
            new_path = first_half + new_part
        '''
        for j in range(len(path1)):
            print("path1",path1[j])
        for j in range(len(path2)):
            print("path2",path2[j])

        for j in range(len(first_half)):
            print("first_half",first_half[j])
        for j in range(len(new_part)):
            print("new_part",new_part[j])
        '''

        return Path(new_path)


    '''
    # 变异过程
    def mutate(self, path, grid_x, grid_y, grid_z, objectives):
        new_path = copy.deepcopy(path)
        for i in range(1, len(new_path)):
            if randint(0, 100) > 100 - self.mutation_rate and new_path[i] not in objectives:
                p = Point(int(random.normal(new_path[i].x, self.std)), int(random.normal(new_path[i].y, self.std)),
                          int(random.normal(new_path[i].z, self.std)))

                if p.x < 0:
                    p.x = 0
                elif p.x >= grid_x:
                    p.x = grid_x - 1

                if p.y < 0:
                    p.y = 0
                elif p.y >= grid_y:
                    p.y = grid_y - 1

                if p.z < 0:
                    p.z = 0
                elif p.z >= grid_z:
                    p.z = grid_z - 1

                if p not in new_path:
                    new_path[i].x = p.x
                    new_path[i].y = p.y
                    new_path[i].z = p.z

        return new_path
    '''

    def mutate(self, path, objectives, Kca):
        # choose the starting point for mutation
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
        tmp = [path.points[starting_point]]
        # sort the objectives randomly
        # shuffle(objectives_)
        while total_obj > 0:
            allowed_point = True
            next_x = 0
            next_y = 0
            next_z = 0
            min_x = prev_x
            max_x = prev_x
            if objectives_[0].x < prev_x:
                min_x = prev_x - 1
            if objectives_[0].x > prev_x:
                max_x = prev_x + 1

            min_y = prev_y
            max_y = prev_y
            if objectives_[0].y < prev_y:
                min_y = prev_y - 1
            if objectives_[0].y > prev_y:
                max_y = prev_y + 1

            min_z = prev_z
            max_z = prev_z
            if objectives_[0].z < prev_z:
                min_z = prev_z - 1
            if objectives_[0].z > prev_z:
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

            prev_x = next_x
            prev_y = next_y
            prev_z = next_z

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
        first_half = copy.deepcopy(path.points[:starting_point])
        next_half = tmp
        new_path = Path(first_half + next_half)

        return new_path

    '''
    # 平滑过程
    def smooth(self, path, objectives):
        new_path = copy.deepcopy(path)
        for i in range(1, len(new_path) - 1):
            if randint(0, 100) > 100 - self.smoothing_rate and new_path[i] not in objectives:
                p = Point(int((new_path[i - 1].x + new_path[i + 1].x) / 2),
                          int((new_path[i - 1].y + new_path[i + 1].y) / 2),
                          int((new_path[i - 1].z + new_path[i + 1].z) / 2))

                if p not in new_path:
                    new_path[i].x = p.x
                    new_path[i].y = p.y
                    new_path[i].z = p.z

        return new_path
    '''

    def tournament_select(self, paths):
        winner_index = 0
        best_fitness = -9999999

        for i in range(self.tournament_size):
            rand_index = randint(0, len(paths) - 1)
            if paths[rand_index].fitness > best_fitness:
                best_fitness = paths[rand_index].fitness
                winner_index = rand_index

        return winner_index

    # 选择方法
    # selection_size 选择的规模
    def select(self, paths, selection_size, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num):
        # 计算fitness之和
        fitness_sum = 0
        for path in paths:
            fitness_sum += self.get_fitness(path, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num)

        # 构建轮盘
        tmp = 0
        roulette_list = []
        for path in paths:
            fitness = self.get_fitness(path, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num)
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
        # 父子融合
        tmp_paths = old_paths
        tmp_paths.extend(new_paths)
        # 优胜劣汰
        quick_sort(tmp_paths)
        return tmp_paths[:population]

    '''
    def get_fitness(self, path, occ_grid):
        fitness_ = 0

        for point in path:
            # The closer to an obstacle, the more points lost
            fitness_ += occ_grid[point.x][point.y][point.z] * 10

        fitness_ += len(path)

        return 1.0 / fitness_
    '''

    def get_fitness(self, path, occ_grid, pri_grid, start, end, sum_privacy, obstacle_num):
        # 系数设定
        safety = 0
        privacy = 0
        flag = 0
        alpha = 1
        beta = 1

        for point in path.points:
            # The closer to an obstacle, the more points lost
            if occ_grid[point.x][point.y][point.z] == 1:
                flag += -1
                safety += occ_grid[point.x][point.y][point.z]
        # print (safety)
        for point in path.points:
            privacy += math.exp(point.ca) * pri_grid[point.x][point.y][point.z]
        # h*exp(-ws-(1/2)*dis^2)
        # print(privacy)
        if sum_privacy == 0:
            privacy = 0
        else :
            privacy = privacy/sum_privacy

        ideal_length = abs(start.x - end.x) + abs(start.y - end.y) + abs(start.z - end.z)
        length = len(path.points)
        efficiency = ideal_length / (length - 1)
        # safety = 1 - safety / obstacle_num  # 可以统计所有的障碍物的数目
        # print(ideal_length,length,efficiency)
        # print(obstacle_num, safety)
        # print(sum_privacy,privacy)
        # print(flag)
        # maximize
        fitness = flag + beta * efficiency + alpha * privacy
        # print(fitness)
        if path.flag < 0:
            path.flag = 1

        return fitness
