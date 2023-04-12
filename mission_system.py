

class mission_system:
    '''
    任务系统，里面包括多个基础任务，高级任务等信息
    '''
    def __init__(self, id=-1) -> None:
        self.id = id # 表示该任务系统的id
        self.worker_num = 0 # 该任务系统的工作台数量
        self.worker_id_list = [] # 该任务系统的工作台id

        self.mission_type = -1 # 该任务系统生成的产品类型

        self.robot_id_list = [] # 当前在该任务系统中工作的机器人的id
        self.thin_path_dict = dict()
        self.wide_path_dict = dict()
        self.task_dict = dict()

        self.utile = -1

    def initialize(self, roboti, worker_list, mission_type, utile, all_route_wide):
        # 得到该任务系统要生成的物品类型
        self.mission_type = mission_type
        self.utile = utile

        # 获取生成该类型物品所需要的工作台的id
        # 1. 如果是生成7产品：
        if mission_type==7:
            worker7_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 7:
                    distance = (roboti.position_x - worker_list[i].position_x)**2 + (roboti.position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker7_id = i

            worker89_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type in [8,9]:
                    distance = (worker_list[worker7_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker7_id].position_x - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker89_id = i

            # 同理找到4，5，6
            worker6_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 6 and worker_list[i].in_mission_system==0:
                    distance = (worker_list[worker7_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker7_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker6_id = i
            
            worker5_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 5 and worker_list[i].in_mission_system==0:
                    distance = (worker_list[worker7_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker7_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker5_id = i            

            worker4_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 4 and worker_list[i].in_mission_system==0:
                    distance = (worker_list[worker7_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker7_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker4_id = i     

            # 找到6所需要的2和3
            worker26_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 2:
                    distance = (worker_list[worker6_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker6_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker26_id = i   

            worker36_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 3:
                    distance = (worker_list[worker6_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker6_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker36_id = i 

            # 找到5所需要的1和3
            worker15_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 1:
                    distance = (worker_list[worker5_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker5_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker15_id = i   

            worker35_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 3:
                    distance = (worker_list[worker5_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker5_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker35_id = i    

            # 找到4所需要的1和2
            worker14_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 1:
                    distance = (worker_list[worker4_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker4_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker14_id = i   

            worker24_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 2:
                    distance = (worker_list[worker4_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker4_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker24_id = i    

            # 1. 添加所有路线
            needed_route = [(worker7_id, worker89_id), 
                            (worker6_id, worker7_id),
                            (worker5_id, worker7_id),
                            (worker4_id, worker7_id),
                            (worker36_id, worker6_id),
                            (worker26_id, worker6_id),
                            (worker15_id, worker5_id),
                            (worker35_id, worker5_id),
                            (worker14_id, worker4_id),
                            (worker24_id, worker4_id)]

            # 2. 添加所有工作台id
            for i in needed_route:
                for j in i:
                    if j not in self.worker_id_list:
                        self.worker_id_list.append(j)
            
            self.worker_num = len(self.worker_id_list)

            self.utile.calc_route(self.worker_id_list, worker_list, all_route_wide)
            # 3. update route

            length = len(needed_route)
            for i in range(length):
                self.thin_path_dict[needed_route[i]] = 0
                self.thin_path_dict[needed_route[i][::-1]] = 0
                self.wide_path_dict[needed_route[i]] = all_route_wide[needed_route[i][0]][needed_route[i][1]]
                self.wide_path_dict[needed_route[i][::-1]] = all_route_wide[needed_route[i][1]][needed_route[i][0]]

            # 4. 记录所有的task以及它们被做的次数
            for i in needed_route:
                self.task_dict[i] = 0

            # 4. 更改用到的工作台的信息
            worker_list[worker4_id].in_mission_system = 1
            worker_list[worker5_id].in_mission_system = 1
            worker_list[worker6_id].in_mission_system = 1

        elif mission_type in [4,5,6]:
            worker456_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == mission_type and worker_list[i].in_mission_system==0:
                    distance = (roboti.position_x - worker_list[i].position_x)**2 + (roboti.position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker456_id = i

            worker9_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 9:
                    distance = (worker_list[worker456_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker456_id].position_x - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker9_id = i
            
            son_dict = {4:(1,2), 5:(1,3), 6:(2,3)}
                  
            worker0456_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == son_dict[mission_type][0]:
                    distance = (worker_list[worker456_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker456_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker0456_id = i   

            worker1456_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == son_dict[mission_type][1]:
                    distance = (worker_list[worker456_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker456_id].position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker1456_id = i        
            # 1. 添加所有路线
            needed_route = [(worker456_id, worker9_id),
                            (worker0456_id, worker456_id),
                            (worker1456_id, worker456_id)]

            # 2. 添加所有工作台id
            for i in needed_route:
                for j in i:
                    if j not in self.worker_id_list:
                        self.worker_id_list.append(j)
            
            self.worker_num = len(self.worker_id_list)

            self.utile.calc_route(self.worker_id_list, worker_list, all_route_wide)
            # 3. update route

            length = len(needed_route)
            for i in range(length):
                self.thin_path_dict[needed_route[i]] = 0
                self.thin_path_dict[needed_route[i][::-1]] = 0
                self.wide_path_dict[needed_route[i]] = all_route_wide[needed_route[i][0]][needed_route[i][1]]
                self.wide_path_dict[needed_route[i][::-1]] = all_route_wide[needed_route[i][1]][needed_route[i][0]]

            # 4. 记录所有的task以及它们被做的次数
            for i in needed_route:
                self.task_dict[i] = 0

            # 4. 更改用到的工作台的信息
            worker_list[worker456_id].in_mission_system = 1

        elif mission_type in [1,2,3]:
            worker123_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == mission_type:
                    distance = (roboti.position_x - worker_list[i].position_x)**2 + (roboti.position_y - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker123_id = i

            worker9_id = -1
            min_dis = 100000
            for i in roboti.reachable_worker_id_list:
                if worker_list[i].worker_type == 9:
                    distance = (worker_list[worker123_id].position_x - worker_list[i].position_x)**2 + (worker_list[worker123_id].position_x - worker_list[i].position_y)**2
                    if distance < min_dis:
                        min_dis = distance
                        worker9_id = i            
            # 1. 添加所有路线
            needed_route = [(worker123_id, worker9_id)]
            # 2. 添加所有工作台id
            for i in needed_route:
                for j in i:
                    if j not in self.worker_id_list:
                        self.worker_id_list.append(j)
            
            self.worker_num = len(self.worker_id_list)

            self.utile.calc_route(self.worker_id_list, worker_list, all_route_wide)
            # 3. update route

            length = len(needed_route)
            for i in range(length):
                self.thin_path_dict[needed_route[i]] = 0
                self.thin_path_dict[needed_route[i][::-1]] = 0
                self.wide_path_dict[needed_route[i]] = all_route_wide[needed_route[i][0]][needed_route[i][1]]
                self.wide_path_dict[needed_route[i][::-1]] = all_route_wide[needed_route[i][1]][needed_route[i][0]]

            # 4. 记录所有的task以及它们被做的次数
            for i in needed_route:
                self.task_dict[i] = 0




    def initialize_special(self, needed_route, all_route_thin, all_route_wide, utile):
        """
        TODO
        """
        # 添加所有路线
        self.utile = utile

        length = len(needed_route)
        for i in range(length):
            self.thin_path_dict[needed_route[i]] = all_route_thin[needed_route[i][0]][needed_route[i][1]]
            self.thin_path_dict[needed_route[i][::-1]] = all_route_thin[needed_route[i][1]][needed_route[i][0]]
            self.wide_path_dict[needed_route[i]] = all_route_wide[needed_route[i][0]][needed_route[i][1]]
            self.wide_path_dict[needed_route[i][::-1]] = all_route_wide[needed_route[i][1]][needed_route[i][0]]

        # 添加所有工作台id
        for i in needed_route:
            for j in i:
                if j not in self.worker_id_list:
                    self.worker_id_list.append(j)
        
        self.worker_num = len(self.worker_id_list)
        # 记录所有的task以及它们被做的次数
        for i in needed_route:
            self.task_dict[i] = 0

    def get_task(self, worker_list, map_structure, all_route_wide):
        """
        首先获得可以执行的task，然后执行其中最少被完成的task
        return:
        -1 表示没找到可以执行的任务
        (3,4) : 表示起止工作台的编号
        """
        suit_task_list = self.get_suit_task(worker_list)
        min_time = 1000000
        min_tk = -1
        for tk in suit_task_list:
            if tk in self.task_dict.keys():
                # TODO
                if self.task_dict[tk] < min_time:
                    min_time = self.task_dict[tk]
                    min_tk = tk

        if min_tk != -1:
            if worker_list[min_tk[0]].worker_type not in [1,2,3]:
                worker_list[min_tk[0]].reserve_production_flag = 1
            else:
                worker_list[min_tk[0]].reserve_production_flag = 0
            
            if worker_list[min_tk[1]].worker_type not in [8,9]:
                worker_list[min_tk[1]].raw_materials[worker_list[min_tk[0]].worker_type] = [1,0,1]
            else:
                worker_list[min_tk[1]].raw_materials[worker_list[min_tk[0]].worker_type] = [1,0,0]

            self.task_dict[min_tk] += 1
            if self.wide_path_dict[min_tk] == 0:
                if all_route_wide[min_tk[0]][min_tk[1]] == 0 or all_route_wide[min_tk[0]][min_tk[1]] == []:
                    route = self.utile.find_route(min_tk[0], min_tk[1], worker_list, map_structure, 1)
                    all_route_wide[min_tk[0]][min_tk[1]] = route
                    all_route_wide[min_tk[1]][min_tk[0]] = route[::-1]
                    self.wide_path_dict[min_tk] = route
                    self.wide_path_dict[min_tk[::-1]] = route[::-1]
                else:
                    route = all_route_wide[min_tk[0]][min_tk[1]]
                    self.wide_path_dict[min_tk] = route
                    self.wide_path_dict[min_tk[::-1]] = route[::-1]

            return min_tk, self.wide_path_dict[min_tk]

        return min_tk, []

    def get_suit_task(self, worker_list):
        """
        获取任务系统中所有可以执行的任务
        """
        suit_task_list = []
        # 找到有成品的工作台，并检查它们的父工作台是否空闲
        for i in self.worker_id_list:
            if (worker_list[i].production_flag == 1 and worker_list[i].reserve_production_flag == 0) or worker_list[i].worker_type in [1,2,3]:
                for j in self.worker_id_list:
                    if worker_list[j].raw_materials[worker_list[i].worker_type] == [1,0,0]:
                        suit_task_list.append((i, j))

        return suit_task_list


    def calculate_time(self, worker_list, root_id):
        '''
        TODO 计算完成该任务系统的总时间
        '''
        pass

    def calculate_income(self):
        '''
        计算完成该任务系统的总收益
        '''
        type_money = {7: 71400, 6: 14900, 5: 14200, 4: 13300, 3: 3400, 2: 3200, 1: 3000}
        return type_money[self.root_type]









































