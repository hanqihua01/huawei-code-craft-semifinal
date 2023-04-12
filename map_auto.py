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

        self.utile = utile()

        self.total_frame = 15000

        # TODO 最后几秒的判断
        self.more_time = 50 # frame


    
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
        系统初始化时，每个机器人选择他们的任务系统
        只需要更新self.robot_list的中robot的状态即可
        '''

        self.utile.initialize(self.map_structure)

        # 1. 让机器人知道自己可以到达的工作台的集合
        # 2. 让机器人知道自己和哪些机器人是可以一起工作的
        self.robot_process()

        # 3. 对机器人0建立任务系统0，并将机器人0加入该任务系统，但是此时不找路径
        flag7 = 0
        m7 = -1
        for roboti in self.robot_list:
            if roboti.reachable_worker_id_list == []:
                roboti.die = 1
            elif roboti.mission_system_id != -1:
                continue
            else:
                flag_list = []
                for i in range(8):
                    flag_list.append(set())
                for worker_id in roboti.reachable_worker_id_list:
                    if self.worker_list[worker_id].worker_type == 7:
                        flag_list[7].add(7)
                    elif self.worker_list[worker_id].worker_type == 6:
                        flag_list[6].add(6)
                        flag_list[7].add(6)
                    elif self.worker_list[worker_id].worker_type == 5:
                        flag_list[5].add(5)
                        flag_list[7].add(5)
                    elif self.worker_list[worker_id].worker_type == 4:
                        flag_list[4].add(4)
                        flag_list[7].add(4)                    
                    elif self.worker_list[worker_id].worker_type == 3:
                        flag_list[3].add(3)
                        flag_list[5].add(3)
                        flag_list[6].add(3)
                        flag_list[7].add(3)
                    elif self.worker_list[worker_id].worker_type == 2:
                        flag_list[2].add(2)
                        flag_list[4].add(2)
                        flag_list[6].add(2)
                        flag_list[7].add(2)
                    elif self.worker_list[worker_id].worker_type == 1:
                        flag_list[1].add(1)
                        flag_list[4].add(1)
                        flag_list[5].add(1)
                        flag_list[7].add(1)
                    elif self.worker_list[worker_id].worker_type == 8:
                        flag_list[7].add(8)
                    elif self.worker_list[worker_id].worker_type == 9:
                        for i in range(1,8):
                            flag_list[i].add(9)
                
                # 首先检查是否可以生产7：
                if len(flag_list[7]) == 9 or (len(flag_list[7])==8 and len(flag_list[7].intersection(set([8,9])))==1):
                    # TODO 说明可以生产7
                    new_mission = mission_system(self.mission_number)
                    new_mission.initialize(roboti, self.worker_list, 7, self.utile, self.all_route_wide)
                    # 看看当前机器人的邻居是否可以加入该任务系统
                    neighbor_id_list = []
                    for robotj in self.robot_list:
                        if robotj.mission_system_id == -1 and robotj.id in roboti.neighbor_robot_id_list:
                            neighbor_id_list.append(robotj.id)

                    # 最多只能3个机器人在这个任务系统中
                    if len(neighbor_id_list) == 4:
                        neighbor_id_list = neighbor_id_list[:-1]
                    
                    for id in neighbor_id_list:
                        self.robot_list[id].die = 0
                        self.robot_list[id].in_task = 0
                        self.robot_list[id].mission_system_id = self.mission_number
                        new_mission.robot_id_list.append(id)
                    
                    flag7 = 1
                    m7 = self.mission_number
                    self.mission_list.append(new_mission)
                    self.mission_number += 1
                    
                    break
        #####################################################################################################################
        for roboti in self.robot_list:
            if roboti.reachable_worker_id_list == []:
                roboti.die = 1
            elif roboti.mission_system_id != -1:
                continue
            else:
                flag_list = []
                for i in range(8):
                    flag_list.append(set())
                for worker_id in roboti.reachable_worker_id_list:
                    if self.worker_list[worker_id].worker_type == 7:
                        flag_list[7].add(7)
                    elif self.worker_list[worker_id].worker_type == 6:
                        flag_list[6].add(6)
                        flag_list[7].add(6)
                    elif self.worker_list[worker_id].worker_type == 5:
                        flag_list[5].add(5)
                        flag_list[7].add(5)
                    elif self.worker_list[worker_id].worker_type == 4:
                        flag_list[4].add(4)
                        flag_list[7].add(4)                    
                    elif self.worker_list[worker_id].worker_type == 3:
                        flag_list[3].add(3)
                        flag_list[5].add(3)
                        flag_list[6].add(3)
                        flag_list[7].add(3)
                    elif self.worker_list[worker_id].worker_type == 2:
                        flag_list[2].add(2)
                        flag_list[4].add(2)
                        flag_list[6].add(2)
                        flag_list[7].add(2)
                    elif self.worker_list[worker_id].worker_type == 1:
                        flag_list[1].add(1)
                        flag_list[4].add(1)
                        flag_list[5].add(1)
                        flag_list[7].add(1)
                    elif self.worker_list[worker_id].worker_type == 8:
                        flag_list[7].add(8)
                    elif self.worker_list[worker_id].worker_type == 9:
                        for i in range(1,8):
                            flag_list[i].add(9)


                # 检查是否可以生产4，5，6
                worker4_list = set()
                worker5_list = set()
                worker6_list = set()
                for i in roboti.reachable_worker_id_list:
                    if self.worker_list[i].worker_type==4 and self.worker_list[i].in_mission_system == 0:
                        worker4_list.add(4)
                    elif self.worker_list[i].worker_type==5 and self.worker_list[i].in_mission_system == 0:
                        worker5_list.add(5)
                    elif self.worker_list[i].worker_type==6 and self.worker_list[i].in_mission_system == 0:
                        worker6_list.add(6)                    
                    elif self.worker_list[i].worker_type==1:
                        worker4_list.add(1)
                        worker5_list.add(1)
                    elif self.worker_list[i].worker_type==2:
                        worker4_list.add(2)
                        worker6_list.add(2)
                    elif self.worker_list[i].worker_type==3:
                        worker5_list.add(3)
                        worker6_list.add(3)
                    elif self.worker_list[i].worker_type==9:
                        worker4_list.add(9)
                        worker5_list.add(9)
                        worker6_list.add(9)
                
                flag = 0
                new_mission = mission_system(self.mission_number)
                if len(worker6_list) == 4:
                    # 说明可以生成6
                    new_mission.initialize(roboti, self.worker_list, 6, self.utile, self.all_route_wide)
                    flag=1
                elif len(worker5_list) == 4:
                    # 说明可以生成5
                    new_mission.initialize(roboti, self.worker_list, 5, self.utile, self.all_route_wide)
                    flag=1
                elif len(worker4_list) == 4:
                    # 说明可以生成4
                    new_mission.initialize(roboti, self.worker_list, 4, self.utile, self.all_route_wide)
                    flag=1

                if flag==1:
                    roboti.die = 0
                    roboti.in_task = 0
                    roboti.mission_system_id = self.mission_number
                    new_mission.robot_id_list.append(roboti.id)
                    self.mission_list.append(new_mission)
                    self.mission_number += 1
                elif flag7 == 1:
                    roboti.die = 0
                    roboti.in_task = 0
                    roboti.mission_system_id = m7
                    self.mission_list[m7].robot_id_list.append(roboti.id)
                else:
                    # 说明无法生产4，5，6
                    # 检查是否可以进行1，2，3的买卖
                    for i in [3,2,1]:
                        if len(flag_list[i]) == 2:
                            #  说明可以进行1，2，3的买卖
                            new_mission = mission_system(self.mission_number)
                            new_mission.initialize(roboti, self.worker_list, i, self.utile, self.all_route_wide)
                            roboti.die = 0
                            roboti.in_task = 0
                            roboti.mission_system_id = self.mission_number
                            new_mission.robot_id_list.append(roboti.id)
                            self.mission_list.append(new_mission)
                            self.mission_number += 1                                  
                            flag=1
                            break

                    if flag==0:
                        roboti.die = 1

        pause = 1

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
                    inst = roboti.go_along_route(roboti.route0, 1, self.robot_list, self.map_structure, self.frame_number)
                else:
                    inst = roboti.go_along_route(roboti.route0, 0, self.robot_list, self.map_structure, self.frame_number)
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
                    inst = roboti.go_along_route(roboti.route1, 1, self.robot_list, self.map_structure, self.frame_number)
                else:
                    inst = roboti.go_along_route(roboti.route1, 0, self.robot_list, self.map_structure, self.frame_number)
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
                        
                        

