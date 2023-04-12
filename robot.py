from math import sqrt, acos, asin, pi, sin, cos, atan

from utile import utile


class robot:
    def __init__(self, id, px, py, sx=0, sy=0, w=0, near_worker_id=-1, production_tpye=0, time_value=0, crash_value=0):
        self.reachable_worker_id_list = [] # 可以到达的工作台的id
        self.neighbor_robot_id_list = [] # 处于一个连通域中的机器人id，包括自己的id

        self.id = id
        self.position_x = px
        self.position_y = py
        self.utile = utile()
        self.gx, self.gy = self.utile.meter2grid(px, py)

        self.speed_x = sx
        self.speed_y = sy
        self.angle_speed = w
        self.near_worker_id = near_worker_id # 表示附近可以进行买卖的工作台id
        self.production_tpye = production_tpye # 携带物品类型,1-7,0表示没有携带物品
        self.time_value = time_value
        self.crash_value = crash_value

        self.mission_system_id = -1 # 指示机器人所属任务系统
        self.in_task = 0 # 0表示机器人不在执行任务，1表示机器人正在执行任务
        self.task = [] # (start工作台，end工作台)
        self.route0 = [] # 机器人到达start工作台
        self.route1 = [] # 从start工作台到达end工作台
        self.die = 0 # 表示机器人是否永久停止运动
        self.is_start = 0 # 表示机器人是否处于route的起点

        self.destination_index = 1 # 目标在route列表中的下标
        self.last_final_destination = (0, 0) # 上一个终点grid坐标
        self.smooth_route = [] # 平滑后路径

        self.avoid_north_wall = False
        self.avoid_south_wall = False
        self.avoid_west_wall = False
        self.avoid_east_wall = False

        self.avoid_north_wall_angle = 0
        self.avoid_south_wall_angle = 0
        self.avoid_west_wall_angle = 0
        self.avoid_east_wall_angle = 0     

        self.avoid_crash = 0b0000
        self.crash_end = 0b0000
        self.by_crash_end = 0b0000

        self.start_frame_num = 0
        self.start_grid_list = []
        self.cur_frame_num = 0
        self.go_back_step = 1
        self.go_back_steps = [1, 1, 1, 1]

        self.go_back_flag = False

    def update_statement(self, state):
        self.near_worker_id = int(state[0])
        self.production_tpye = int(state[1])
        self.time_value = float(state[2])
        self.crash_value = float(state[3])
        self.angle_speed = float(state[4])
        self.speed_x = float(state[5])
        self.speed_y = float(state[6])
        self.orientation = float(state[7])
        self.position_x = float(state[8])
        self.position_y = float(state[9])

    def transport_production(self, task, route0, route1):
        '''
        领取运送成品的任务 未验证
        ''' 
        self.task = task
        self.route0 = route0
        self.route1 = route1
        self.is_start = 1

        self.in_task = 1

    def calculate_ang(self, x1, y1, x2, y2):
        d1 = sqrt(x1**2 + y1**2)
        ux1 = x1 / d1
        uy1 = y1 / d1
        d2 = sqrt(x2**2 + y2**2)
        ux2 = x2 / d2
        uy2 = y2 / d2
        return acos(ux1*ux2 + uy1*uy2)

    def smoothing_route(self, route):
        '''
        路径平滑处理
        '''
        # ----------参数列表----------
        threshold = pi / 8 # 两路线夹角小于该阈值，就把中间点删除掉
        # ----------参数列表----------

        ret = []
        length = len(route)
        if (length <= 2):
            return route
        first = 0
        second = 1
        third = 2
        ret.append(route[first])
        while (third < length):
            x0, y0 = self.utile.grid2meter(route[first][0], route[first][1])
            x1, y1 = self.utile.grid2meter(route[second][0], route[second][1])
            x2, y2 = self.utile.grid2meter(route[third][0], route[third][1])
            ang_dis = self.calculate_ang(x1-x0, y1-y0, x2-x1, y2-y1)
            if (ang_dis < threshold):
                second = third
                third += 1
                if (third >= length):
                    ret.append(route[second])
                    break
            else:
                ret.append(route[second])
                first = second
                second = third
                third += 1
                if (third >= length):
                    ret.append(route[second])
                    break
        return ret
      
    def go_along_route(self, route, start, robot_list, map_structure, frame_num, map_id):
        '''
        返回一个指令列表，让机器人靠近route的终点
        route: [(start grid x, start grid y), (), (), ... , (end grid x, end grid y)]
        route的终点就是列表的最后一项
        '''
        rbt_x = self.position_x
        rbt_y = self.position_y

        if (start == 1):
            self.destination_index = 1
            self.last_final_destination = self.utile.grid2meter(route[0][0], route[0][1])
            self.smooth_route = route
        (des_x, des_y) = self.utile.grid2meter(self.smooth_route[self.destination_index][0], self.smooth_route[self.destination_index][1])
        distance = sqrt((des_x-rbt_x)**2 + (des_y-rbt_y)**2)
        if (distance < 0.4):
            self.destination_index += 1
        if (self.destination_index >= len(self.smooth_route)):
            self.destination_index = len(self.smooth_route) - 1

        # 回退模块
        self.cur_frame_num = frame_num
        go_back_time = 400
        if (map_id == 1):
            go_back_time = 100

        if (self.cur_frame_num - self.start_frame_num >= go_back_time):

            go_back_step = self.go_back_step
            if (self.production_tpye == 0):
                r = 0.45
            else:
                r = 0.53
            crash_flag = False
            for i in range(0, 4):
                if (i != self.id):
                    robot = robot_list[i]
                    robot_px = robot.position_x
                    robot_py = robot.position_y
                    dis = sqrt((robot_px - rbt_x) ** 2 + (robot_py - rbt_y) ** 2)
                    if (robot.production_tpye == 0):
                        rr = 0.45
                    else:
                        rr = 0.53
                    if (dis < r + rr + 0.05):
                        crash_flag = True
                        if (self.go_back_steps[robot.id] > robot.go_back_steps[self.id]):
                            go_back_step = self.go_back_steps[robot.id]
                            self.go_back_steps[robot.id] += 1
                        else:
                            go_back_step = robot.go_back_steps[self.id]
                            robot.go_back_steps[self.id] += 1
            if (crash_flag == False):
                go_back_step = self.go_back_step
                self.go_back_step += 1

            self.destination_index -= go_back_step

            self.start_frame_num = self.cur_frame_num
            if (self.destination_index < 0):
                self.destination_index = 1
        else:
            (grid_x, grid_y) = self.utile.meter2grid(rbt_x, rbt_y)
            if ((grid_x, grid_y) not in self.start_grid_list):
                self.start_grid_list.clear()
                self.start_grid_list.append((grid_x, grid_y))
                self.start_grid_list.append((grid_x - 1, grid_y - 1))
                self.start_grid_list.append((grid_x - 1, grid_y))
                self.start_grid_list.append((grid_x - 1, grid_y + 1))
                self.start_grid_list.append((grid_x, grid_y - 1))
                self.start_grid_list.append((grid_x, grid_y + 1))
                self.start_grid_list.append((grid_x + 1, grid_y - 1))
                self.start_grid_list.append((grid_x + 1, grid_y))
                self.start_grid_list.append((grid_x + 1, grid_y + 1))
                self.start_frame_num = self.cur_frame_num

        (des_x, des_y) = self.utile.grid2meter(self.smooth_route[self.destination_index][0], self.smooth_route[self.destination_index][1])
        if (self.destination_index == len(self.smooth_route) - 1):
            is_sharp_bend = True
        else:
            (next_x, next_y) = self.utile.grid2meter(self.smooth_route[self.destination_index+1][0], self.smooth_route[self.destination_index+1][1])
            turn_angle = self.calculate_ang(des_x-rbt_x, des_y-rbt_y, next_x-des_x, next_y-des_y)
            if (turn_angle > pi / 6): # FIXME
                is_sharp_bend = True
            else:
                is_sharp_bend = False
        if (map_id == 1):
            result = self.run_to_des1(des_x, des_y, is_sharp_bend, self.last_final_destination[0], self.last_final_destination[1], robot_list, map_structure)        
        else:
            result = self.run_to_des(des_x, des_y, is_sharp_bend, self.last_final_destination[0], self.last_final_destination[1], robot_list, map_structure)
        final_speed = result[0]
        final_angle = result[1]
        instruction_list = []
        instruction_list.append('forward %d %f\n' % (self.id, final_speed))
        instruction_list.append('rotate %d %f\n' % (self.id, final_angle))
        return instruction_list

    def run_to_des(self, des_x, des_y, is_sharp_bend, last_final_des_x, last_final_des_y, robot_list, map_structure):
        '''
        制定运动策略，返回线速度和角速度
        '''
        final_speed = 0
        final_angle = 0
        
        cx = self.position_x
        cy = self.position_y
        wx = des_x
        wy = des_y
        dx = wx - cx
        dy = wy - cy
        d = sqrt(dx ** 2 + dy ** 2)
        udx = dx / d
        udy = dy / d

        vx = self.speed_x
        vy = self.speed_y
        v = sqrt(vx ** 2 + vy ** 2)
        ori = self.orientation
        uvx = cos(ori)
        uvy = sin(ori)

        theta = acos(udx * uvx + udy * uvy)
        judge = uvx * udy - uvy * udx
        omega_sml = theta if judge > 0 else -theta
        omega_big = pi if judge > 0 else -pi
        tmp = 0.2 / d
        if tmp>1:
            tmp = 0.999999
        threshold = asin(tmp) # FIXME

        if self.production_tpye == 0:
            r = 0.45
        else:
            r = 0.53

        if (sqrt((cx-last_final_des_x)**2 + (cy-last_final_des_y)**2) < 0.4): # FIXME
            if (d > 1.91):
                if (theta < threshold):
                    final_speed = 6
                    final_angle = 0
                else:
                    final_speed = 0
                    if (theta - threshold > pi / 6): # FIXME
                        final_angle = omega_big
                    else:
                        final_angle = omega_big / 2 # FIXME
            else: 
                if (theta < threshold):
                    # 解法1
                    if (is_sharp_bend):
                        final_speed = 4 # FIXME
                    else:
                        final_speed = 6 # FIXME
                    # 解法2
                    # final_speed = 1.25
                    final_angle = 0
                else:
                    final_speed = 0
                    if (theta - threshold > pi / 6): # FIXME
                        final_angle = omega_big
                    else:
                        final_angle = omega_big / 2 # FIXME
        else:
            if (d > 1.91):
                if (theta < threshold):
                    final_speed = 6
                    final_angle = 0
                else: # FIXME
                    # 解法1
                    # if (theta < pi / 2):
                    #     final_speed = 3
                    #     final_angle = omega_sml * 2
                    # else:
                    #     final_speed = 1.25
                    #     final_angle = omega_big
                    # 解法2
                    if (theta - threshold > pi / 6): # FIXME
                        final_speed = 1.25 # FIXME
                        final_angle = omega_big
                    else:
                        final_speed = 3 # FIXME
                        final_angle = omega_big / 2 # FIXME
            else:
                if (theta < threshold):
                    # 解法1
                    if (is_sharp_bend):
                        final_speed = 4 # FIXME
                    else:
                        final_speed = 6 # FIXME
                    # 解法2
                    # final_speed = 1.25
                    final_angle = 0
                else:
                    final_speed = 1.25
                    final_angle = omega_big



        # 避免机器人撞墙撞死
        dis_to_wall_threshold = r + 0.05
        dis_to_avoid_wall = r + 0.45
        if (sqrt((cx-last_final_des_x)**2 + (cy-last_final_des_y)**2) >= 0.4): # FIXME
            north = self.utile.meter2grid(cx, cy + dis_to_wall_threshold)
            south = self.utile.meter2grid(cx, cy - dis_to_wall_threshold)
            west = self.utile.meter2grid(cx - dis_to_wall_threshold, cy)
            east = self.utile.meter2grid(cx + dis_to_wall_threshold, cy)
            north_big = self.utile.meter2grid(cx, cy + dis_to_avoid_wall)
            south_big = self.utile.meter2grid(cx, cy - dis_to_avoid_wall)
            west_big = self.utile.meter2grid(cx - dis_to_avoid_wall, cy)
            east_big = self.utile.meter2grid(cx + dis_to_avoid_wall, cy)

            if (north[0] >= 0):
                if (map_structure[north[0]][north[1]] == -1 and pi / 4 < ori < 3 * pi / 4):
                    if (self.avoid_north_wall == False):
                        self.avoid_north_wall = True
                        self.avoid_north_wall_angle = omega_big
            else:
                final_speed = 0
                final_angle = omega_big
            if (self.avoid_north_wall):
                if (0 < ori < pi):
                    final_speed = 0
                    final_angle = self.avoid_north_wall_angle
                else:
                    if (map_structure[north[0]][north[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_north_wall = False

            if (south[0] <= 99):
                if (map_structure[south[0]][south[1]] == -1 and -3 * pi / 4 < ori < -pi / 4):
                    if (self.avoid_south_wall == False):
                        self.avoid_south_wall = True
                        self.avoid_south_wall_angle = omega_big
            else:
                final_speed = 0
                final_angle = omega_big
            if (self.avoid_south_wall):
                if (-pi < ori < 0):
                    final_speed = 0
                    final_angle = self.avoid_south_wall_angle
                else:
                    if (map_structure[south[0]][south[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_south_wall = False

            if (west[1] >= 0):
                if (map_structure[west[0]][west[1]] == -1 and (3 * pi / 4 < ori <= pi or -pi <= ori < -3 * pi / 4)):
                    if (self.avoid_west_wall == False):
                        self.avoid_west_wall = True
                        self.avoid_west_wall_angle = omega_big
            else:
                final_speed = 0
                final_angle = omega_big
            if (self.avoid_west_wall):
                if (pi / 2 < ori <= pi or -pi <= ori < -pi / 2):
                    final_speed = 0
                    final_angle = self.avoid_west_wall_angle
                else:
                    if (map_structure[west[0]][west[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_west_wall = False

            if (east[1] <= 99):
                if (map_structure[east[0]][east[1]] == -1 and -pi / 4 < ori < pi / 4):
                    if (self.avoid_east_wall == False):
                        self.avoid_east_wall = True
                        self.avoid_east_wall_angle = omega_big
            else:
                final_speed = 0
                final_angle = omega_big
            if (self.avoid_east_wall):
                if (-pi / 2 < ori < pi / 2):
                    final_speed = 0
                    final_angle = self.avoid_east_wall_angle
                else:
                    if (map_structure[east[0]][east[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_east_wall = False

        # # 避免机器人怼一起撞死
        for i in range(0, 4):
            if (i != self.id):
                robot = robot_list[i]
                robot_px = robot.position_x
                robot_py = robot.position_y
                dis = sqrt((robot_px - cx) ** 2 + (robot_py - cy) ** 2)
                if (robot.production_tpye == 0):
                    rr = 0.45
                else:
                    rr = 0.53
                if (dis < r + rr + 0.05):
                    # FIXME
                    # final_angle = omega_big
                    if (cx < robot.position_x):
                        if (ori > 0):
                            final_angle += (pi + 0.3125) / 4.5
                        else:
                            final_angle += -(pi + 0.3125) / 4.5
                    else:
                        if (ori > 0):
                            final_angle += -(pi + 0.3125) / 4.5
                        else:
                            final_angle += (pi + 0.3125) / 4.5

        return (final_speed, final_angle)

    def run_to_des1(self, des_x, des_y, is_sharp_bend, last_final_des_x, last_final_des_y, robot_list, map_structure):
        '''
        FIXME 返回一个指令列表，让机器人靠近目标工作台
        '''

        instruction_list = [] # 机器人self的两条指令
        final_speed = 0 # 设置机器人self的线速度
        final_angle = 0 # 设置机器人self的角速度
        
        cx = self.position_x # self的x坐标
        cy = self.position_y # self的y坐标
        wx = des_x # target_worker的x坐标
        wy = des_y # target_worker的y坐标
        dx = wx - cx # self与target_worker的x坐标差值
        dy = wy - cy # self与target_worker的y坐标差值
        d = sqrt(dx ** 2 + dy ** 2) # self与target_worker的距离
        udx = dx / d # x坐标差值单位化
        udy = dy / d # y坐标差值单位化

        vx = self.speed_x # self的x速度
        vy = self.speed_y # self的y速度
        v = sqrt(vx ** 2 + vy ** 2) # self的线速度
        ori = self.orientation # self的朝向
        if v != 0:
            uvx = vx / v # self的x速度单位化
            uvy = vy / v # self的y速度单位化
        else:
            uvx = cos(ori) # self的x虚拟速度单位化
            uvy = sin(ori) # self的y虚拟速度单位化

        theta = acos(udx * uvx + udy * uvy) # self的速度向量与目标向量的夹角，范围0-pi
        multi = uvx * udy - uvy * udx # 判断self的速度向量与目标向量夹角正负
        omega = theta if multi > 0 else -theta # self需要旋转的角速度（小）
        omega1 = pi if multi > 0 else -pi # self需要旋转的角速度（大）
        threshold = asin(0.4 / d) # self朝向目标的阈值

        if self.production_tpye == 0:
            r = 0.45 # self没有携带物品时的半径
        else:
            r = 0.53 # self携带物品时的半径

        '''
        不考虑机器人相撞，设置线速度和角速度
        '''
        # ----------可调节参数列表begin----------
        robot_boundary = 0.75 # 机器人位于边界判定阈值
        next_to_boundary = 0.75 - r # 机器人紧贴边界的判定阈值
        speed_in_boundary = 0.5 # 位于边界内且不紧贴边界时线速度

        speed_far_small_angle = 6.0 # 离目标远且不朝向目标且夹角小于二分之派时线速度
        speed_far_big_angle = 1.25 # 离目标远且不朝向目标且夹角大于二分之派时线速度

        worker_boundary = 1.25 # 工作台位于边界判定阈值
        speed_short_worker_in_boundary_in_direction = 2.0 # 离目标近且朝向目标且工作台在边界时线速度
        # -----------可调节参数列表end-----------
        # FIXME 如何制定边界减速条件
        if (cy >= 50 - robot_boundary and ori > 0 and ori < pi): # self位于上边界
            if (cy + r + next_to_boundary >= 50): # self紧贴上边界
                final_speed = 0.0
                final_angle = omega1
            else: # self不紧贴上边界
                final_speed = speed_in_boundary
                if (theta < threshold): # self朝向目标
                    final_angle = 0.0
                else: # self未朝向目标
                    final_angle = omega1
        elif (cy <= robot_boundary and ori > -pi and ori < 0): # self位于下边界
            if (cy <= r + next_to_boundary): # self紧贴下边界
                final_speed = 0.0
                final_angle = omega1
            else: # self不紧贴下边界
                final_speed = speed_in_boundary
                if (theta < threshold): # self朝向目标
                    final_angle = 0.0
                else: # self未朝向目标
                    final_angle = omega1
        elif (cx >= 50 - robot_boundary and ori > -pi / 2 and ori < pi / 2): # self位于右边界
            if (cx + r + next_to_boundary >= 50): # self紧贴右边界
                final_speed = 0.0
                final_angle = omega1
            else: # self不紧贴右边界
                final_speed = speed_in_boundary
                if (theta < threshold): # self朝向目标
                    final_angle = 0.0
                else: # self未朝向目标
                    final_angle = omega1
        elif (cx <= robot_boundary and ((ori > pi / 2 and ori <= pi) or (ori >= -pi and ori < -pi / 2))): # self位于左边界
            if (cx <= r + next_to_boundary): # self紧贴左边界
                final_speed = 0.0
                final_angle = omega1
            else: # self不紧贴左边界
                final_speed = speed_in_boundary
                if (theta < threshold): # self朝向目标
                    final_angle = 0.0
                else: # self未朝向目标
                    final_angle = omega1
        else: # self不位于任何边界
            if (d > 1.91): # self离目标较远
                if (theta < threshold): # self朝向目标
                    final_speed = 6.0
                    final_angle = 0.0
                else: # self未朝向目标
                    if (theta < pi / 2): # self速度与目标夹角小于pi / 2，设置小角速度
                        final_speed = speed_far_small_angle
                        final_angle = omega * 2
                    else: # self速度与目标夹角大于pi / 2，设置大角速度
                        final_speed = speed_far_big_angle
                        final_angle = omega1
            else: # self离目标较近
                if (wx > worker_boundary and wx < 50 - worker_boundary and wy > worker_boundary and wy < 50 - worker_boundary): # 工作台不在边界
                    if (theta < threshold): # self朝向目标
                        final_speed = 6.0
                        final_angle = 0.0
                    else: # self未朝向目标
                        final_speed = 1.25
                        final_angle = omega1
                else: # 工作台在边界
                    if (theta < threshold): # self朝向目标
                        final_speed = speed_short_worker_in_boundary_in_direction
                        final_angle = 0.0
                    else: # self未朝向目标
                        final_speed = 1.25
                        final_angle = omega1

        # 避免机器人撞墙撞死
        dis_to_wall_threshold = r + 0.05
        dis_to_avoid_wall = r + 0.45
        if (sqrt((cx-last_final_des_x)**2 + (cy-last_final_des_y)**2) >= 0.4): # FIXME
            north = self.utile.meter2grid(cx, cy + dis_to_wall_threshold)
            south = self.utile.meter2grid(cx, cy - dis_to_wall_threshold)
            west = self.utile.meter2grid(cx - dis_to_wall_threshold, cy)
            east = self.utile.meter2grid(cx + dis_to_wall_threshold, cy)
            north_big = self.utile.meter2grid(cx, cy + dis_to_avoid_wall)
            south_big = self.utile.meter2grid(cx, cy - dis_to_avoid_wall)
            west_big = self.utile.meter2grid(cx - dis_to_avoid_wall, cy)
            east_big = self.utile.meter2grid(cx + dis_to_avoid_wall, cy)

            if (north[0] >= 0):
                if (map_structure[north[0]][north[1]] == -1 and pi / 4 < ori < 3 * pi / 4):
                    if (self.avoid_north_wall == False):
                        self.avoid_north_wall = True
                        self.avoid_north_wall_angle = omega1
            else:
                final_speed = 0
                final_angle = omega1
            if (self.avoid_north_wall):
                if (0 < ori < pi):
                    final_speed = 0
                    final_angle = self.avoid_north_wall_angle
                else:
                    if (map_structure[north[0]][north[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_north_wall = False

            if (south[0] <= 99):
                if (map_structure[south[0]][south[1]] == -1 and -3 * pi / 4 < ori < -pi / 4):
                    if (self.avoid_south_wall == False):
                        self.avoid_south_wall = True
                        self.avoid_south_wall_angle = omega1
            else:
                final_speed = 0
                final_angle = omega1
            if (self.avoid_south_wall):
                if (-pi < ori < 0):
                    final_speed = 0
                    final_angle = self.avoid_south_wall_angle
                else:
                    if (map_structure[south[0]][south[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_south_wall = False

            if (west[1] >= 0):
                if (map_structure[west[0]][west[1]] == -1 and (3 * pi / 4 < ori <= pi or -pi <= ori < -3 * pi / 4)):
                    if (self.avoid_west_wall == False):
                        self.avoid_west_wall = True
                        self.avoid_west_wall_angle = omega1
            else:
                final_speed = 0
                final_angle = omega1
            if (self.avoid_west_wall):
                if (pi / 2 < ori <= pi or -pi <= ori < -pi / 2):
                    final_speed = 0
                    final_angle = self.avoid_west_wall_angle
                else:
                    if (map_structure[west[0]][west[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_west_wall = False

            if (east[1] <= 99):
                if (map_structure[east[0]][east[1]] == -1 and -pi / 4 < ori < pi / 4):
                    if (self.avoid_east_wall == False):
                        self.avoid_east_wall = True
                        self.avoid_east_wall_angle = omega1
            else:
                final_speed = 0
                final_angle = omega1
            if (self.avoid_east_wall):
                if (-pi / 2 < ori < pi / 2):
                    final_speed = 0
                    final_angle = self.avoid_east_wall_angle
                else:
                    if (map_structure[east[0]][east[1]] == -1): # FIXME
                        final_speed = 1.25
                        final_angle = 0
                    else:
                        self.avoid_east_wall = False

        return (final_speed, final_angle)