import sys
import copy

from robot import robot
from worker import worker
from frame_statement import frame_statement
from mission_system import mission_system
from utile import utile

class map:
    def __init__(self):
        self.robot_number = 0
        self.robot_list = [] # store robots

        self.leaf_worker_type = [1,2,3]
        self.end_worker_type = [8, 9]
        self.middle_worker_type = [4,5,6,7]

        self.worker_number = 0
        self.worker_list = [] # store workers
        self.worker_group_list = []
        for i in range(10):
            self.worker_group_list.append([]) # 以组的形式存储每种类型工作台的id，一共9种，第0位空着

        self.frame_number = 0
        self.frame_list = [] # 保存有所有frame statement的列表

        self.mission_number = 0
        self.mission_list = []

        self.map_structure = []  # 0: empty, 1-9: worker, -1 wall
        for i in range(100):
            self.map_structure.append([0]*100)

        self.all_route_wide = [] # 保存所有工作台之间的路线
        self.all_route_thin = []
        self.needed_route0 = [(1,6),(4,6),(1,8),(0,8),(4,7),(0,7),(6,5),(6,3),(6,2),(8,5),(8,3),(8,2),(7,5),(7,3),(7,2)]
        self.needed_route1 = []
        self.needed_route2 = []
        self.needed_route3 = []

        self.utile = utile()

        self.total_frame = 15000

        # TODO 最后几秒的判断
        self.more_time = 0.2
        self.map_id = 1

    def read_route(self):
        # TODO
        lines_thin = [[(2,8),(90,8),(96,2),(97,2),(1,6)],
                        [(2,8),(5,8),(10,3),(13,3),(1,4)],
                        [(2,8),(52,8),(97,53),(98,53),(1,8)],
                        [(2,8),(2,49),(1,50),(1,0)],
                        [(2,8),(91,97),(97,97),(1,7)],
                        [(2,8),(8,8),(13,3),(23,3),(1,5)],
                        [(2,8),(2,12),(3,13),(1,3)],
                        [(2,8),(2,90),(1,2)],
                        [(97,2),(14,2),(13,3),(6,4)],
                        [(97,2),(97,52),(98,53),(6,8)],
                        [(97,2),(50,2),(2,50),(1,50),(6,0)],
                        [(97,2),(97,81),(98,82),(98,96),(97,97),(6,7)],
                        [(97,2),(24,2),(23,3),(6,5)],
                        [(97,2),(15,2),(4,13),(3,13),(6,3)],
                        [(97,2),(9,90),(2,90),(6,2)],
                        [(13,3),(47,3),(97,53),(98,53),(4,8)],
                        [(13,3),(13,37),(1,49),(1,50),(4,0)],
                        [(13,3),(97,87),(97,97),(4,7)],
                        [(13,3),(23,3),(4,5)],
                        [(13,3),(3,13),(4,3)],
                        [(13,3),(13,78),(2,89),(2,90),(4,2)],
                        [(98,53),(46,53),(43,50),(1,50),(8,0)],
                        [(98,53),(98,96),(97,97),(8,7)],
                        [(98,53),(74,53),(24,3),(23,3),(8,5)],
                        [(98,53),(46,53),(6,13),(3,13),(8,3)],
                        [(98,53),(45,53),(8,90),(2,90),(8,2)],
                        [(1,50),(49,50),(96,97),(97,97),(0,7)],
                        [(1,50),(7,44),(7,20),(23,4),(23,3),(0,5)],
                        [(1,50),(1,16),(3,14),(3,13),(0,3)],
                        [(1,50),(1,89),(2,90),(0,2)],
                        [(97,97),(97,91),(87,81),(87,68),(23,4),(23,3),(7,5)],
                        [(97,97),(85,85),(78,85),(6,13),(3,13),(7,3)],
                        [(97,97),(10,97),(3,90),(2,90),(7,2)],
                        [(23,3),(13,13),(3,13),(5,3)],
                        [(23,3),(23,18),(22,19),(22,66),(2,86),(2,90),(5,2)],
                        [(3,13),(3,79),(2,80),(2,90),(3,2)]]

        for l in lines_thin:
            route = l
            tmp = route[-1]
            route = route[:-1]
            if route != []:
                self.all_route_thin[tmp[0]][tmp[1]] = route
                self.all_route_thin[tmp[1]][tmp[0]] = route[::-1]

        # TODO
        lines_wide = [[(2,8),(90,8),(96,2),(97,2),(1,6)],
                        [(2,8),(5,8),(10,3),(13,3),(1,4)],
                        [(2,8),(52,8),(97,53),(98,53),(1,8)],
                        [(2,8),(2,49),(1,50),(1,0)],
                        [(2,8),(91,97),(97,97),(1,7)],
                        [(2,8),(8,8),(13,3),(23,3),(1,5)],
                        [(2,8),(2,12),(3,13),(1,3)],
                        [(2,8),(2,90),(1,2)],
                        [(97,2),(14,2),(13,3),(6,4)],
                        [(97,2),(97,52),(98,53),(6,8)],
                        [(97,2),(50,2),(2,50),(1,50),(6,0)],
                        [(97,2),(97,81),(98,82),(98,96),(97,97),(6,7)],
                        [(97,2),(24,2),(23,3),(6,5)],
                        [(97,2),(15,2),(4,13),(3,13),(6,3)],
                        [(97,2),(9,90),(2,90),(6,2)],
                        [(13,3),(47,3),(97,53),(98,53),(4,8)],
                        [(13,3),(13,37),(1,49),(1,50),(4,0)],
                        [(13,3),(97,87),(97,97),(4,7)],
                        [(13,3),(23,3),(4,5)],
                        [(13,3),(3,13),(4,3)],
                        [(13,3),(13,78),(2,89),(2,90),(4,2)],
                        [(98,53),(46,53),(43,50),(1,50),(8,0)],
                        [(98,53),(98,96),(97,97),(8,7)],
                        [(98,53),(74,53),(24,3),(23,3),(8,5)],
                        [(98,53),(46,53),(6,13),(3,13),(8,3)],
                        [(98,53),(45,53),(8,90),(2,90),(8,2)],
                        [(1,50),(49,50),(96,97),(97,97),(0,7)],
                        [(1,50),(7,44),(7,20),(23,4),(23,3),(0,5)],
                        [(1,50),(1,16),(3,14),(3,13),(0,3)],
                        [(1,50),(1,89),(2,90),(0,2)],
                        [(97,97),(97,91),(87,81),(87,68),(23,4),(23,3),(7,5)],
                        [(97,97),(85,85),(78,85),(6,13),(3,13),(7,3)],
                        [(97,97),(10,97),(3,90),(2,90),(7,2)],
                        [(23,3),(13,13),(3,13),(5,3)],
                        [(23,3),(23,18),(22,19),(22,66),(2,86),(2,90),(5,2)],
                        [(3,13),(3,79),(2,80),(2,90),(3,2)]]

        for l in lines_wide:
            route = l
            tmp = route[-1]
            route = route[:-1]
            if route != []:
                self.all_route_wide[tmp[0]][tmp[1]] = route
                self.all_route_wide[tmp[1]][tmp[0]] = route[::-1]

    def get_route(self, worker_id_list, save_path):
        # TODO
        wide_route_list = []
        thin_route_list = []
        length = len(worker_id_list)
        for i in range(length-1):
            for j in range(i+1,length):
                route_wide = self.utile.find_route(worker_id_list[i], worker_id_list[j], self.worker_list, self.map_structure, wide=1)
                route_wide.append((worker_id_list[i],worker_id_list[j]))
                wide_route_list.append(route_wide)

                route_thin = self.utile.find_route(worker_id_list[i], worker_id_list[j], self.worker_list, self.map_structure, wide=0)
                route_thin.append((worker_id_list[i], worker_id_list[j]))
                thin_route_list.append(route_thin)

        wide_route_txt = open("wide_" + save_path, "w")
        wide_route_txt.write("[")
        for route in wide_route_list:
            tmp = []
            for i in route:
                tmp.append("({},{})".format(i[0], i[1]))
            wide_route_txt.write("[" + ",".join(tmp) + "]," + "\n")
        wide_route_txt.write("]")
        wide_route_txt.close()

        thin_route_txt = open("thin_" + save_path, "w")
        thin_route_txt.write("[")
        for route in thin_route_list:
            tmp = []
            for i in route:
                tmp.append("({},{})".format(i[0], i[1]))
            thin_route_txt.write("[" + ",".join(tmp) + "]," + "\n")
        thin_route_txt.write("]")
        thin_route_txt.close()
    
    def get_worker_id_list(self):
        needed_route = self.needed_route0 + self.needed_route1 + self.needed_route2 + self.needed_route3
        wil = []
        for p in needed_route:
            for w in p:
                if w not in wil:
                    wil.append(w)
        return wil

    def read_map(self):
        # 此处读入地图，并进行预处理
        position_x = 0.25
        position_y = 49.75
        line = input()
        while line != "OK":
            for i in line:
                # 机器人类型
                if i == "A":
                    new_robot = robot(id=self.robot_number, px=position_x, py=position_y)
                    self.robot_list.append(new_robot)
                    self.robot_number += 1
                
                # 工作台类型
                elif i in "123456789":
                    gx,gy = self.utile.meter2grid(position_x, position_y)
                    self.map_structure[gx][gy] = int(i)
                    new_worker = worker(id=self.worker_number, px=position_x, py=position_y, gx=gx, gy=gy, worker_type=int(i))
                    self.worker_list.append(new_worker)
                    self.worker_group_list[int(i)].append(new_worker.id)
                    self.worker_number += 1
                
                elif i == "#":
                    gx,gy = self.utile.meter2grid(position_x, position_y)
                    self.map_structure[gx][gy] = -1

                position_x += 0.5
            position_x = 0.25
            position_y -= 0.5
            line = input()
        
        # 读入地图后进行一些预先处理，目前all route什么也没有记录
        for i in range(self.worker_number):
            self.all_route_wide.append([0]*self.worker_number)
            self.all_route_thin.append([0]*self.worker_number)
        self.pre_calculate() 

    def robot_process(self):
        """
        # 1. 让机器人知道自己可以到达的工作台的集合
        # 2. 让机器人知道自己和哪些机器人是可以一起工作的
        """
        for roboti in self.robot_list:
            rgx, rgy = roboti.gx, roboti.gy
            rbti_d_type = self.utile.get_domain_type(rgx,rgy,self.utile.connected_domain_wide)
            for workeri in self.worker_list:
                wgx, wgy = workeri.grid_x, workeri.grid_y
                wkr_d_type = self.utile.get_domain_type(wgx,wgy,self.utile.connected_domain_wide)
                if wkr_d_type == rbti_d_type:
                    roboti.reachable_worker_id_list.append(workeri.id)

            for robotj in self.robot_list:
                rbtj_d_type = self.utile.get_domain_type(robotj.gx, robotj.gy, self.utile.connected_domain_wide)
                if rbtj_d_type==rbti_d_type:
                    roboti.neighbor_robot_id_list.append(robotj.id)


    def pre_calculate(self):
        '''
        TODO 目前针对地图1
        系统初始化时，每个机器人选择他们的任务系统
        只需要更新self.robot_list的中robot的状态即可
        '''
        self.utile.initialize(self.map_structure)
        
        route_path = "route.txt"
        # 1. get all possible route and save them in route_path
        worker_id_list = self.get_worker_id_list()
        # self.get_route(worker_id_list, route_path)

        # 2. read all path from route_path to self.all_route
        self.read_route()

        # 3. use self.needed_route0 to create a mission
        new_mission0 = mission_system(id=0)
        new_mission0.initialize_special(self.needed_route0, self.all_route_thin, self.all_route_wide, self.utile)
        self.mission_list.append(new_mission0)
        self.mission_number += 1

        # 4. use self.needed_route1 to create a mission
        # new_mission1 = mission_system(id=1)
        # new_mission1.initialize_special(self.needed_route1, self.all_route_thin, self.all_route_wide, self.utile)
        # self.mission_list.append(new_mission1)
        # self.mission_number += 1

        
        # add robot 0 to mission 0
        self.robot_list[0].die = 1
        self.robot_list[0].in_task = 0
        self.robot_list[0].mission_system_id = 0
        new_mission0.robot_id_list.append(0)

        # add robot 1 to mission 1
        self.robot_list[1].die = 1
        self.robot_list[1].in_task = 0
        self.robot_list[1].mission_system_id = 1
        new_mission0.robot_id_list.append(1)

        # add robot 2 to mission 0
        self.robot_list[2].die = 1
        self.robot_list[2].in_task = 0
        self.robot_list[2].mission_system_id = 0
        new_mission0.robot_id_list.append(2)

        # add robot 3 to mission 0
        self.robot_list[3].die = 0
        self.robot_list[3].in_task = 0
        self.robot_list[3].mission_system_id = 0
        new_mission0.robot_id_list.append(3)
        


    def add_new_frame(self, frame_id, money):
        new_frame = frame_statement(id=frame_id, money=money)
        worker_number = int(input())
        # 读取工作台的状态
        for index in range(worker_number):
            line = input()
            parts = line.split(" ")
            self.worker_list[index].update_statement(parts)
            new_frame.worker_statement.append(parts)

        # 读取机器人的状态
        for index in range(self.robot_number):
            line = input()
            parts = line.split(" ")
            self.robot_list[index].update_statement(parts)
            new_frame.robot_statement.append(parts)
        
        self.frame_number += 1
        self.frame_list.append(new_frame)
        if self.frame_number > 1:
            old_frame = self.frame_list[self.frame_number-2]
            self.frame_list[self.frame_number-2] = 0
            del old_frame
            
        # TODO debug专用断点，可以在任何帧停下来
        if frame_id in [391]:
            pause = 1
        ok = input()
        if ok != "OK":
            raise RuntimeError
    
    def print_out_instruction(self):
        '''
        # FIXME 
        # 此时决策模块已经完成
        # 根据机器人领取的任务，得到每个机器人的行动指令，并输出
        '''
        # 根据函数decision中每个机器人得到的任务，得到机器人的执行指令，并放入frame_statement中
        for roboti in self.robot_list:
            if roboti.die == 1 or roboti.in_task == 0:
                # 表示该机器人处于无任务状态，应该发出指令，停止机器人运行
                self.stop_robot(roboti)
            else:
                self.instruct_robot(roboti)

        sys.stdout.write('%d\n' % self.frame_number)
        current_frame_instructions = self.frame_list[self.frame_number - 1].instructions
        for instruction in current_frame_instructions:
            sys.stdout.write(instruction)

    def stop_robot(self, roboti):
        '''
        # FIXME 将停止机器人运行的指令放入frame_statement中
        注意指令为一个字符串，以\n结尾
        '''
        # 此时应该要让机器人停止
        # if roboti.production_tpye != 0:
        #     raise RuntimeError
        
        # 添加指令到当前帧中
        current_frame = self.frame_list[self.frame_number - 1]
        current_frame.instructions.append("forward " + str(int(roboti.id)) + " 0\n") # 前进速度为0
        current_frame.instructions.append("rotate " + str(int(roboti.id)) + " 0\n") # 旋转速度为0


    def instruct_robot(self, roboti):
        '''
        # FIXME 针对机器人目前执行的任务，将相应指令放入frame_statement中 很多细节没有验证
        注意指令为一个字符串，以\n结尾
        '''
        current_frame = self.frame_list[self.frame_number - 1]
        total_instructions = []
        if roboti.production_tpye==0:
            # 1. 机器人执行route0
            if roboti.near_worker_id != roboti.task[0]:
                if roboti.is_start == 1:
                    roboti.is_start = 0
                    inst = roboti.go_along_route(roboti.route0, 1, self.robot_list, self.map_structure, self.frame_number,self.map_id)
                else:
                    inst = roboti.go_along_route(roboti.route0, 0, self.robot_list, self.map_structure, self.frame_number,self.map_id)
                total_instructions += inst
            else:
                # 买了东西后先停止
                if (self.worker_list[roboti.task[0]].production_flag == 0 or self.worker_list[roboti.task[0]].reserve_production_flag == 0) and self.worker_list[roboti.task[0]].worker_type not in [1,2,3]:
                    # 没有买到，说明运动模块出现了问题
                    raise RuntimeError
                else:
                    flag = self.time_enough(roboti.id, roboti.task[1])
                    if flag == 1:
                        total_instructions.append("buy {}\n".format(int(roboti.id)))
                        roboti.go_back_step = 1
                        roboti.go_back_steps = [1, 1, 1, 1]
                        total_instructions.append("forward {} 0\n".format(int(roboti.id)))
                        total_instructions.append("rotate {} 0\n".format(int(roboti.id)))
                        self.worker_list[roboti.task[0]].reserve_production_flag = 0
                        self.worker_list[roboti.task[0]].production_flag = 0
                        roboti.is_start = 1 # 目前已经处于route1的起点
                    else:
                        total_instructions.append("forward {} 0\n".format(int(roboti.id)))
                        total_instructions.append("rotate {} 0\n".format(int(roboti.id)))                        

        else:
            # 2. 机器人执行route1
            if roboti.near_worker_id != roboti.task[1]:
                if roboti.is_start == 1:
                    roboti.is_start = 0
                    inst = roboti.go_along_route(roboti.route1, 1, self.robot_list, self.map_structure, self.frame_number,self.map_id)
                else:
                    inst = roboti.go_along_route(roboti.route1, 0, self.robot_list, self.map_structure, self.frame_number,self.map_id)
                total_instructions += inst
            else:
                # 卖出东西后先停止
                total_instructions.append("sell {}\n".format(int(roboti.id)))
                roboti.go_back_step = 1
                roboti.go_back_steps = [1, 1, 1, 1]
                total_instructions.append("forward {} 0\n".format(int(roboti.id)))
                total_instructions.append("rotate {} 0\n".format(int(roboti.id)))
                if self.worker_list[roboti.task[1]].worker_type in [8,9]:
                    self.worker_list[roboti.task[1]].raw_materials[roboti.production_tpye] = [1,0,0]
                else:
                    self.worker_list[roboti.task[1]].raw_materials[roboti.production_tpye] = [1,1,0]
                roboti.in_task = 0


        # 将控制roboti的指令加入total_instructions中
        current_frame.instructions += total_instructions

    def time_enough(self, robot_id, destination_worker_id):
        '''
        判断机器人买下成品后，是否有足够时间送到
        假设为最快速度+0.5s之内可以到达，则任务可以到达
        '''

        roboti = self.robot_list[robot_id]
        route = self.all_route_wide[roboti.near_worker_id][destination_worker_id]
        distance = 0
        length = len(route)
        for i in range(1,length):
            s = route[i-1]
            e = route[i]
            distance += ((s[0]-e[0])**2 + (s[1]-e[1])**2)**0.5
        
        if (distance / 8) * 50 + self.more_time < self.total_frame - self.frame_number:
            return 1
        return 0


    def decision(self):
        '''
        # TODO 决策模块
        在该模块执行完成以后，机器人如果处于没有任务的状态，则应该停止运行
        如果机器人处于有任务的状态，则应该针对它的任务输出相应的指令
        '''
        for roboti in self.robot_list:
            if roboti.die == 1:
                continue
            else:
                if roboti.in_task == 1:
                    continue
                else:
                    current_mission = self.mission_list[roboti.mission_system_id]
                    # 1.1 任务系统给出当前最合适的任务
                    # 这里开始时可能会进行路径搜索，会发生卡顿
                    task, route1 = current_mission.get_task(self.worker_list, self.map_structure, self.all_route_wide)
                    if task == -1:
                        # 机器人会继续等
                        continue
                    else:
                        # 1.2 需要知道机器人如何到达start工作台
                        route0 = -1
                        if roboti.near_worker_id != -1:
                            if self.all_route_thin[roboti.near_worker_id][task[0]] != 0 and self.all_route_thin[roboti.near_worker_id][task[0]] != []:
                                route0 = self.all_route_thin[roboti.near_worker_id][task[0]]

                        if route0 == -1:
                            # 没有现成的路径，需要找一条新路
                            start_x, start_y = self.utile.meter2grid(roboti.position_x, roboti.position_y)
                            end_x, end_y = self.utile.meter2grid(self.worker_list[task[0]].position_x, self.worker_list[task[0]].position_y)

                            # FIXME 假定一定可以找到，这里非常消耗时间
                            route0 = self.utile.find_route_thin(start_x, start_y, end_x, end_y, self.map_structure)

                            # 检查是否需要记录该路径
                            if roboti.near_worker_id != -1:
                                self.all_route_thin[roboti.near_worker_id][task[0]] = route0
                                self.all_route_thin[task[0]][roboti.near_worker_id] = route0[::-1]

                        roboti.transport_production(task, route0, route1)
                        
                        

