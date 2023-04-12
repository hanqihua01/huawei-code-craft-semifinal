

class worker:
    def __init__(self, id, px, py, gx, gy, worker_type, time_remaining=-1, raw_materials=0, production_flag=0):
        self.id = id # 就是该工作台在worker list中的序号
        self.position_x = px
        self.position_y = py
        self.grid_x = gx
        self.grid_y = gy

        self.worker_type = worker_type # 工作台类型，int
        self.son_type = [] # 可能的儿子的类型，也就是原材料的类型
        self.father_type = [] # 可能的父亲的类型，也就是可以收购该工作台成品的工作台类型

        # 在每个任务系统中的son与father的id
        self.son_id_in_mission = [] # [mission_id, [son_id, ...]]
        self.father_id_in_mission = [] # [mission_id, [father_id, ...]]
        self.in_mission_system = 0 # 是否是已经在任务系统中了.

        self.time_remaining = time_remaining # -1：不在生产中；0：成品格占用造成阻塞；>=0：生产剩余帧数
        self.raw_materials = [] # 二进制表示的原材料格状态 [是否有该原材料格子，是否已有该原材料，该原材料是否已经从其它地方预定好了]
        for i in range(10):
            self.raw_materials.append([0,0,0])
        
        self.production_flag = production_flag # 是否有成品
        self.reserve_production_flag = 0 # 成品是否被预定了

        self.initialize()
    
    def initialize(self):
        if self.worker_type == 9:
            self.son_type = [1,2,3,4,5,6,7]
        elif self.worker_type == 8:
            self.son_type = [7]
        elif self.worker_type == 7:
            self.son_type = [4,5,6]
            self.father_type = [8, 9]
        elif self.worker_type == 6:
            self.son_type = [2,3]
            self.father_type = [7,9]
        elif self.worker_type == 5:
            self.son_type = [1,3]
            self.father_type = [7,9]
        elif self.worker_type == 4:
            self.son_type = [1,2]
            self.father_type = [7,9]
        elif self.worker_type == 3:
            self.father_type = [5,6,9]
        elif self.worker_type == 2:
            self.father_type = [4,6,9]
        elif self.worker_type == 1:
            self.father_type = [4,5,9]
        else:
            raise RuntimeError

        # 指明自己需要的原材料的类型
        for materials_type in self.son_type:
            self.raw_materials[materials_type][0] = 1


    def update_statement(self, state):
        # 下面三个可以不更新
        # self.worker_type = int(state[0])
        # self.position_x = float(state[1])
        # self.position_y = float(state[2])

        self.time_remaining = int(state[3])

        raw_materials_statement = int(state[4])
        tmp_str = str(bin(raw_materials_statement))[2:][::-1]
        for i in range(10-len(tmp_str)):
            tmp_str += "0"
        for raw_materials_id in range(10):
            if tmp_str[raw_materials_id] == "1":
                self.raw_materials[raw_materials_id][1] = 1
            else:
                self.raw_materials[raw_materials_id][1] = 0
        
        self.production_flag = int(state[5])

    