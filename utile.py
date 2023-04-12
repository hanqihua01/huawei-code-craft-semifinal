import copy
import math
# from turtle import distance
import numpy as np
import matplotlib.pyplot as plt
import time

from worker import worker


class utile:
    def __init__(self):
        self.Map_wide = -1
        self.Map_thin = -1
        self.Map_thin_box_around = -1
        self.Map_wide_box_around = -1
        self.connected_domain_wide = -1
        self.connected_domain_thin = -1
        self.min_heap = heap()
        self.worker_id = -1
        self.woker_num = -1
        # self.all_routes = -1
        self.woker_pos = -1

    def calc_route(self, worker_id_list, worker_list, all_route_wide):
        for i in worker_id_list:
            DJ = Dijikstra(worker_list[i].grid_x, worker_list[i].grid_y)
            all_route_wide[i] = DJ.Dijikstra_Get_Path(
                self.Map_wide, self.min_heap, self.woker_num, self.worker_id)
        return

    def initialize(self, map_structure):
        Map_wide = self.get_Map_wide(map_structure)
        self.Map_wide = Map_wide

        Map_thin = self.get_Map_thin(map_structure)
        self.Map_thin = Map_thin

        connected_domain_wide = self.get_connected_domain(Map_wide)
        self.connected_domain_wide = connected_domain_wide
        for i in range(100):
            for j in range(100):
                self.get_domain_type(i, j, self.connected_domain_wide)
        connected_domain_thin = self.get_connected_domain(Map_thin)
        self.connected_domain_thin = connected_domain_thin
        for i in range(100):
            for j in range(100):
                self.get_domain_type(i, j, self.connected_domain_thin)
        for i in range(20010):
            node = AStar_node()
            self.min_heap.heapq.append(node)

        # 初始化所有路径
        self.worker_id = []
        self.woker_num = -1
        for i in range(100):
            self.worker_id.append([])
            for j in range(100):
                if map_structure[int(i)][int(j)] != -1 and map_structure[int(i)][int(j)] != 0:
                    self.woker_num += 1
                    self.worker_id[i].append(self.woker_num)
                else:
                    self.worker_id[i].append(-1)
        self.woker_num += 1
        return

    def meter2grid(self, px, py):
        gridx = 99 - int(py*2)
        gridy = int(px*2)
        return gridx, gridy

    def grid2meter(self, gx, gy):
        px = gy/2+0.25
        py = (99-gx)/2 + 0.25
        return px, py

    def get_Map_wide(self, map_structure):
        Next_xy = [[-1, -1], [-1, 0], [-1, 1],
                   [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-2, 0], [2, 0], [0, 2], [0, -2]]
        Add = [1, 2, 1, 2, 1, 2, 1, 2, 1, 1, 1, 1]
        Map = []
        for i in range(100):
            Map.append([])
            for j in range(100):
                Map[i].append(0)
        for i in range(100):
            for j in range(100):
                if map_structure[i][j] == -1:
                    Map[int(i)][int(j)] += 2
                    for k in range(8):
                        ni = i+Next_xy[k][0]
                        nj = j+Next_xy[k][1]
                        if ni < 0 or ni > 99 or nj < 0 or nj > 99:
                            continue
                        # if map_structure[int(ni)][int(nj)] != 0 and map_structure[int(ni)][int(nj)] != -1:
                        #    continue
                        Map[int(ni)][int(nj)] += Add[k]
                elif i == 0 or i == 99 or j == 0 or j == 99:
                    Map[int(i)][int(j)] += 2
        for i in range(100):
            for j in range(100):
                if Map[int(i)][int(j)] < 2:
                    Map[int(i)][int(j)] = '.'
                else:
                    if map_structure[int(i)][int(j)] != 0 and map_structure[int(i)][int(j)] != -1 and Map[i][j] <= 4:
                        Map[i][j] = '.'
                    else:
                        Map[i][j] = '#'
        return Map

    def get_Map_thin(self, map_structure):
        Next_xy = [[-1, 0], [1, 0], [0, 1], [0, -1]]
        Map = []
        for i in range(100):
            Map.append([])
            for j in range(100):
                Map[i].append(0)
        for i in range(100):
            for j in range(100):
                if map_structure[i][j] == -1:
                    Map[int(i)][int(j)] += 2
                    for k in range(4):
                        ni = i+Next_xy[k][0]
                        nj = j+Next_xy[k][1]
                        if ni < 0 or ni > 99 or nj < 0 or nj > 99:
                            continue
                        # if map_structure[int(ni)][int(nj)] != 0 and map_structure[int(ni)][int(nj)] != -1:
                        #    continue
                        Map[int(ni)][int(nj)] += 2
                elif i == 0 or i == 99 or j == 0 or j == 99:
                    Map[int(i)][int(j)] += 2
        for i in range(100):
            for j in range(100):
                if Map[int(i)][int(j)] < 2:
                    Map[int(i)][int(j)] = '.'
                else:
                    if map_structure[int(i)][int(j)] != 0 and map_structure[int(i)][int(j)] != -1 and Map[i][j] <= 4:
                        Map[i][j] = '.'
                    else:
                        Map[i][j] = '#'
        return Map

    def get_domain_type(self, x, y, connected_domain):
        num1 = copy.copy(connected_domain[int(x)][int(y)])
        num2 = x*100+y
        if num1 == num2:
            return num1
        connected_domain[int(x)][int(y)] = self.get_domain_type(
            num1//100, num1 % 100, connected_domain)
        return copy.copy(connected_domain[int(x)][int(y)])

    def get_connected_domain(self, Map):
        # Map可以是Map_thin,Map_wide
        # 使用这个函数分别生成connected_domain_thin,connected_domain_wide
        connected_domain = []
        Next_xy = [[-1, -1], [-1, 0], [-1, 1],
                   [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]
        for i in range(100):
            connected_domain.append([])
            for j in range(100):
                connected_domain[i].append(i*100+j)
        for i in range(100):
            for j in range(100):
                if Map[int(i)][int(j)] == '#':
                    continue
                for k in range(8):
                    ni = i+Next_xy[k][0]
                    nj = j+Next_xy[k][1]
                    if ni < 0 or ni > 99 or nj < 0 or nj > 99 or Map[int(ni)][int(nj)] == '#':
                        continue
                    num1 = self.get_domain_type(i, j, connected_domain)
                    num2 = self.get_domain_type(ni, nj, connected_domain)
                    if num1 == num2:
                        continue
                    connected_domain[int(
                        num1//100)][int(num1 % 100)] = num2
        return connected_domain

    def find_route_wide(self, start_grid_x, start_grid_y, end_grid_x, end_grid_y, map_structure):
        """
        map_structure: dim=(100,100), 0: empty, 1-7: worker, -1 wall
        # 要求机器人即使携带物品也能通过
        return: [(start_grid_x, start_grid_y), (x1, y1), (x2,y2), ... , (end_grid_x, end_grid_y)]
        """
        num1 = self.get_domain_type(
            start_grid_x, start_grid_y, self.connected_domain_wide)
        num2 = self.get_domain_type(
            end_grid_x, end_grid_y, self.connected_domain_wide)
        if num1 != num2:
            # print("haha,get you!")
            return []
        AStar_body = AStar(start_grid_x, start_grid_y)
        route = AStar_body.AStar_Get_Path(
            self.Map_wide, end_grid_x, end_grid_y, self.min_heap, 1)
        if (len(route) == 0):
            return []
        np_route = np.array(route)
        tmp0 = np_route[:, 0]
        tmp1 = np_route[:, 1]
        tmp2 = np.stack([tmp1, tmp0], 1).tolist()
        return tmp2

    def find_route_thin(self, start_grid_x, start_grid_y, end_grid_x, end_grid_y, map_structure):

        num1 = self.get_domain_type(
            start_grid_x, start_grid_y, self.connected_domain_thin)
        num2 = self.get_domain_type(
            end_grid_x, end_grid_y, self.connected_domain_thin)
        if num1 != num2:
            return []

        AStar_body = AStar(start_grid_x, start_grid_y)
        route = AStar_body.AStar_Get_Path(
            self.Map_thin, end_grid_x, end_grid_y, self.min_heap, 0)

        if (len(route) == 0):
            return []
        np_route = np.array(route)
        tmp0 = np_route[:, 0]
        tmp1 = np_route[:, 1]
        tmp2 = np.stack([tmp1, tmp0], 1).tolist()
        return tmp2

    def find_route(self, start_id, end_id, worker_list, map_structure, wide):
        if wide:
            route = self.find_route_wide(worker_list[start_id].grid_x, worker_list[start_id].grid_y,
                                         worker_list[end_id].grid_x, worker_list[end_id].grid_y, map_structure)
        else:
            route = self.find_route_thin(worker_list[start_id].grid_x, worker_list[start_id].grid_y,
                                         worker_list[end_id].grid_x, worker_list[end_id].grid_y, map_structure)
        # print(route)
        return route

    def draw_one_route(self, route, map_structure):
        fig1 = plt.figure()
        plt.axis("equal")
        wall_x = []
        wall_y = []
        worker_x = []
        worker_y = []
        for i in range(100):
            for j in range(100):
                if map_structure[i][j] == -1:
                    wall_x.append(i)
                    wall_y.append(j)
                elif map_structure[i][j] in [1, 2, 3, 4, 5, 6, 7, 8, 9]:
                    worker_x.append(i)
                    worker_y.append(j)

        plt.scatter(np.array(wall_y), 99-np.array(wall_x),
                    marker=".", c="black",)
        plt.scatter(np.array(worker_y), 99 -
                    np.array(worker_x), marker="o", c="blue")

        np_route = np.array(route)
        # plt.scatter(np_route[:, 1], 99-np_route[:, 0], s=5, marker="*", c="red")
        if len(np_route) > 0:
            plt.plot(np_route[:, 1], 99-np_route[:, 0], c="red")
        plt.show()

    def draw_domain_map(self, connected_domain):

        x = []
        y = []
        value = []
        for i in range(100):
            for j in range(100):
                x.append(j)
                y.append(99-i)
                value.append(connected_domain[i][j])

        fig1 = plt.figure()
        plt.axis("equal")

        plt.scatter(x, y, s=10, marker=".", c=value, cmap="hsv")
        for i in range(100):
            print(connected_domain[i])
        plt.show()


class heap:
    # 待验证
    # 小根堆
    def __init__(self):
        self.heapq = ["?"]  # 0号元素不能用
        self.size = 0
        return

    def HeadAdjust(self, k):
        # 下溯
        self.heapq[0] = self.heapq[k]
        i = k*2
        while i <= self.size:
            if i < self.size and self.heapq[i+1] < self.heapq[i]:
                i = i+1
            if not (self.heapq[i] < self.heapq[0]):
                break
            self.heapq[k] = self.heapq[i]
            k, i = i, k*2
        self.heapq[k] = self.heapq[0]
        return

    def TailAdjust(self, k):
        # 上溯
        while k > 1:
            if self.heapq[k] < self.heapq[k//2]:
                self.heapq[k//2], self.heapq[k] = self.heapq[k], self.heapq[k//2]
                k = k//2
            else:
                break
        return

    def InsertHeap(self, val):
        # 插入元素
        self.size = self.size+1
        self.heapq[self.size] = val
        self.TailAdjust(self.size)
        return

    def PopHeap(self):
        # 取出堆顶元素
        if self.size <= 0:
            return -1
        val = self.heapq[1]
        self.heapq[1] = self.heapq[self.size]
        # self.heapq.pop()
        self.size = self.size-1
        if self.size > 0:
            self.HeadAdjust(1)
        return val

    def Empty(self):
        if self.size == 0:
            return True
        return False

    def Print_h(self):
        i = 1
        print("Hallo!")
        while i <= self.size:
            print(self.heapq[i])
            i = i+1
        print("Byby")


class AStar_node:
    # A*节点
    def __init__(self, cur_x=-1, cur_y=-1, cur_val=0, future_val=0, father_x=-1, father_y=-1):
        self.cur_x = cur_x
        self.cur_y = cur_y
        self.cur_val = cur_val
        self.future_val = future_val
        self.sum_val = cur_val+future_val
        self.father_x = father_x
        self.father_y = father_y

        self.direction = -1
        self.change = 0
        return

    def __lt__(self, someother):
        # 重定义小于符号
        temp1 = self.sum_val
        temp2 = someother.sum_val
        # ESP = 0.01
        if temp1 == temp2:  # temp1 < temp2+ESP and temp1 > temp2-ESP:  # temp1=temp2
            if self.change < someother.change:
                return True
            else:
                return False
        if temp1 < temp2:
            return True
        else:
            return False


class AStar:
    def __init__(self, position_x, position_y):
        self.position_x = position_x
        self.position_y = position_y
        self.father = []
        for i in range(100):
            self.father.append([])
            for j in range(100):
                self.father[i].append([-1, -1])
        return

    def AStar_future_cost(self, cur_x, cur_y, target_x, target_y):
        # 未来路径估算函数
        # return fabs(fabs(cur_x-target_x)-fabs(cur_y-target_y))*0.5+0.707*min(fabs(cur_x-target_x), fabs(cur_y-target_y))
        # return math.fabs(cur_x-target_x)+math.fabs(cur_y-target_y)
        return math.hypot(cur_x-target_x, cur_y-target_y)  # 当前网格和终点距离

    def AStar_Get_Path(self, Map, target_x, target_y, min_heap, wide):
        # A*求最短路下一步
        # 数组visited记录当前节点是否被访问
        min_heap.size = 0
        visited = []
        direction = []
        Add_xy = [[[-1, 0], [1, 0]], [], [[-1, 0], [1, 0]],
                  [], [[1, 0], [-1, 0]], [], [[1, 0], [-1, 0]], []]
        for i in range(100):
            visited.append([])
            direction.append([])
            for j in range(100):
                visited[i].append(False)
                direction[i].append(-1)
        # 将物理坐标转换为在字符地图中的坐标
        cur_x = self.position_x+0
        cur_y = self.position_y+0
        # 向堆中加入第一个节点
        cur_val = 0
        future_val = self.AStar_future_cost(cur_x, cur_y, target_x, target_y)

        # min_heap = heap()  # 小根堆
        cur_node = AStar_node(cur_x=cur_x, cur_y=cur_y,
                              cur_val=cur_val, future_val=future_val)
        min_heap.InsertHeap(cur_node)
        # 辅助数组，用于求当前节点向周围八个方向走一步所到达的节点坐标和距离
        Next_xy = [[-1, -1], [-1, 0], [-1, 1],
                   [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]
        Next_dis = [math.sqrt(2), 1, math.sqrt(2), 1,
                    math.sqrt(2), 1, math.sqrt(2), 1]

        while not min_heap.Empty():

            # 取出当前最小路径节点
            cur_node = min_heap.PopHeap()
            if visited[int(cur_node.cur_x)][int(cur_node.cur_y)]:
                continue
            self.father[int(cur_node.cur_x)][int(cur_node.cur_y)] = [
                cur_node.father_x, cur_node.father_y]
            visited[int(cur_node.cur_x)][int(cur_node.cur_y)] = True
            direction[int(cur_node.cur_x)][int(
                cur_node.cur_y)] = cur_node.direction
            if cur_node.cur_x == target_x and cur_node.cur_y == target_y:
                break
            for i in range(8):
                # 更新下一个节点的位置
                next_node = AStar_node()
                next_node.cur_x = cur_node.cur_x+Next_xy[i][0]
                next_node.cur_y = cur_node.cur_y+Next_xy[i][1]
                # 如果已访问则不会再次访问
                # 判断下一个节点是否可达
                # 下一节点超出边界，或者本身是墙

                if (next_node.cur_x < 0 or next_node.cur_x > 99 or next_node.cur_y < 0 or next_node.cur_y > 99 or Map[int(next_node.cur_x)][int(next_node.cur_y)] == '#'):
                    continue
                if wide and ((i & 1) == 0):
                    cx = cur_node.cur_x+Add_xy[i][0][0]
                    cy = cur_node.cur_y+Add_xy[i][0][1]
                    nx = next_node.cur_x+Add_xy[i][1][0]
                    ny = next_node.cur_y+Add_xy[i][1][1]
                    if (Map[int(cx)][int(cy)] == '#' and Map[int(nx)][int(ny)] == '#'):
                        continue
                if visited[int(next_node.cur_x)][int(next_node.cur_y)]:
                    continue
                # 设置next_node其他信息
                next_node.father_x = cur_node.cur_x
                next_node.father_y = cur_node.cur_y
                next_node.direction = i+0
                if next_node.direction != cur_node.direction:
                    next_node.change = cur_node.change+1
                else:
                    next_node.change = cur_node.change

                # 更新下一节点的其他信息
                next_node.cur_val = cur_node.cur_val+Next_dis[i]
                next_node.future_val = self.AStar_future_cost(
                    cur_x=next_node.cur_x, cur_y=next_node.cur_y, target_x=target_x, target_y=target_y)
                next_node.sum_val = next_node.cur_val+next_node.future_val

                min_heap.InsertHeap(val=next_node)
        point = [target_x, target_y]
        '''
        res_List = []
        while point[0] != -1:
            res_List.append([copy.copy(point[1]), copy.copy(point[0])])
            point = self.father[int(point[0])][int(point[1])]
        if len(res_List) == 1:
            return []
        '''
        pre_direction = -2
        res_List = []
        while point[0] != -1:
            if pre_direction != direction[int(point[0])][int(point[1])]:
                res_List.append([point[1], point[0]])
            pre_direction = direction[point[0]][point[1]]
            point = self.father[int(point[0])][int(point[1])]
        if len(res_List) == 1:
            return []
        else:
            if res_List[len(res_List)-1][0] != self.position_y or res_List[len(res_List)-1][1] != self.position_x:
                res_List.append([self.position_y, self.position_x])
            res_List.reverse()
        return res_List


class Dijikstra_node:
    # A*节点
    def __init__(self, cur_x=-1, cur_y=-1, cur_val=0, father_x=-1, father_y=-1):
        self.cur_x = cur_x
        self.cur_y = cur_y
        self.cur_val = cur_val
        self.father_x = father_x
        self.father_y = father_y

        self.direction = -1
        self.change = 0
        return

    def __lt__(self, someother):
        # 重定义小于符号
        temp1 = self.cur_val
        temp2 = someother.cur_val
        # ESP = 0.01
        if temp1 == temp2:  # temp1 < temp2+ESP and temp1 > temp2-ESP:  # temp1=temp2
            if self.change < someother.change:
                return True
            else:
                return False
        if temp1 < temp2:
            return True
        else:
            return False


class Dijikstra:
    def __init__(self, position_x, position_y):
        self.position_x = position_x
        self.position_y = position_y
        self.father = []
        for i in range(100):
            self.father.append([])
            for j in range(100):
                self.father[i].append([-1, -1])
        return

    def Dijikstra_Get_Path(self, Map, min_heap, worker_num, worker_id):
        # Dijkstra求最短路
        # 数组visited记录当前节点是否被访问
        # 辅助数组，用于求当前节点向周围八个方向走一步所到达的节点坐标和距离
        Next_xy = [[-1, -1], [-1, 0], [-1, 1],
                   [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]
        Next_dis = [math.sqrt(2), 1, math.sqrt(2), 1,
                    math.sqrt(2), 1, math.sqrt(2), 1]
        Add_xy = [[[-1, 0], [1, 0]], [], [[-1, 0], [1, 0]],
                  [], [[1, 0], [-1, 0]], [], [[1, 0], [-1, 0]], []]
        min_heap.size = 0
        visited = []
        direction = []
        routes = []
        distance = []
        for i in range(100):
            visited.append([])
            direction.append([])
            distance.append([])
            for j in range(100):
                visited[i].append(False)
                direction[i].append(-1)
                distance[i].append(500)
        distance[int(self.position_x)][int(self.position_y)] = 0
        for i in range(worker_num):
            routes.append([])
        # 将物理坐标转换为在字符地图中的坐标
        cur_x = self.position_x+0
        cur_y = self.position_y+0

        # 向堆中加入第一个节点
        cur_val = 0
        # min_heap = heap()  # 小根堆
        cur_node = Dijikstra_node(cur_x=cur_x, cur_y=cur_y,
                                  cur_val=cur_val)
        min_heap.InsertHeap(cur_node)
        # find_worker_route_num = 0
        while not min_heap.Empty():

            # 取出当前最小路径节点
            cur_node = min_heap.PopHeap()
            if visited[int(cur_node.cur_x)][int(cur_node.cur_y)]:
                continue
            visited[int(cur_node.cur_x)][int(cur_node.cur_y)] = True
            self.father[int(cur_node.cur_x)][int(cur_node.cur_y)] = [
                cur_node.father_x, cur_node.father_y]
            direction[int(cur_node.cur_x)][int(
                cur_node.cur_y)] = cur_node.direction
            # 计算一条路径
            ID = worker_id[int(cur_node.cur_x)][int(cur_node.cur_y)]
            # print(ID)
            if ID != -1:
                point = [cur_node.cur_x, cur_node.cur_y]
                pre_direction = -2
                res_List = routes[int(ID)]
                while point[0] != -1:
                    if pre_direction != direction[int(point[0])][int(point[1])]:
                        res_List.append([point[1], point[0]])
                    pre_direction = direction[point[0]][point[1]]
                    point = self.father[int(point[0])][int(point[1])]
                if len(res_List) == 1:
                    res_List = []
                else:
                    if res_List[len(res_List)-1][0] != self.position_y or res_List[len(res_List)-1][1] != self.position_x:
                        res_List.append([self.position_y, self.position_x])
                    res_List.reverse()
                if (len(res_List) != 0):
                    np_route = np.array(res_List)
                    tmp0 = np_route[:, 0]
                    tmp1 = np_route[:, 1]
                    tmp2 = np.stack([tmp1, tmp0], 1).tolist()
                    routes[int(ID)] = tmp2
            # find_worker_route_num += 1
            # if find_worker_route_num == need_to_find_worker_num:
            #    break
            for i in range(8):
                # 更新下一个节点的位置
                next_node = Dijikstra_node()
                next_node.cur_x = cur_node.cur_x+Next_xy[i][0]
                next_node.cur_y = cur_node.cur_y+Next_xy[i][1]
                # 如果已访问则不会再次访问
                # 判断下一个节点是否可达
                # 下一节点超出边界，或者本身是墙

                if (next_node.cur_x < 0 or next_node.cur_x > 99 or next_node.cur_y < 0 or next_node.cur_y > 99 or Map[int(next_node.cur_x)][int(next_node.cur_y)] == '#'):
                    continue
                if (i & 1) == 0:
                    cx = cur_node.cur_x+Add_xy[i][0][0]
                    cy = cur_node.cur_y+Add_xy[i][0][1]
                    nx = next_node.cur_x+Add_xy[i][1][0]
                    ny = next_node.cur_y+Add_xy[i][1][1]
                    if (Map[int(cx)][int(cy)] == '#' and Map[int(nx)][int(ny)] == '#'):
                        continue

                # 更新下一节点的路径长
                next_node.cur_val = cur_node.cur_val+Next_dis[i]
                if distance[int(next_node.cur_x)][int(next_node.cur_y)] <= next_node.cur_val:
                    continue
                distance[int(next_node.cur_x)][int(
                    next_node.cur_y)] = next_node.cur_val
                # 设置next_node其他信息
                next_node.father_x = cur_node.cur_x
                next_node.father_y = cur_node.cur_y
                next_node.direction = i+0
                if next_node.direction != cur_node.direction:
                    next_node.change = cur_node.change+1
                else:
                    next_node.change = cur_node.change

                min_heap.InsertHeap(val=next_node)
        # print(distance)
        return routes
