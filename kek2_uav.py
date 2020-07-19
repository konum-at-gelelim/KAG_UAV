from base.base_uav import BaseUAV
import math
import random
import util
import time


class KekUAV(BaseUAV):

    def initialize(self):
        self.iteration_count = 0
        self.home = None
        self.take_off = False
        self.last_state = None
        self.dispatch_is_okey = False
        self.formation_phase = None
        self.target_position = None
        self.uav_count = self.params['uav_count']
        self.guide_location = None
        self.a_b = None
        self.a_k = None
        self.formation_type = None
        self.u_b = None
        self.u_k = None
        self.dump = None
        self.formation_id = None
        self.pick_formation_id = True
        self.formation = {'arrow': [], 'prism': []}
        self.speed = [0.0, 0.0]
        self.operation_phase = 0
        self.loop_time = None
        self.loop_location = [0.0, 0.0]
        self.direction = [0.0, 0.0]
        self.pre_sim_time = 0.0
        self.heading = None
        self.lock_heading = False
        # k = 1 unit of [*_speed] /  1 uint of [sim_time]
        #change of location -> (speed * time) * k
        self.k = 0.0005144444

    def act(self):
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        if self.home == None:
            self.home = (self.pose[0], self.pose[1], 100.0)
            self.pre_location = [self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1]]
            self.uav_id = int(self.uav_id)
            self.pre_sim_time = self.uav_msg['sim_time']
            self.heading = self.uav_msg['active_uav']['heading'] % 360.0
        if self.operation_phase == 3:
            self.move_to_target(self.home)
        elif self.operation_phase == 1:
            if not self.uav_msg['uav_guide']['dispatch']:
                self.get_formation_data()
                if self.formation_type == 'arrow':
                    self.formation['arrow'] = self.arrow_gen(self.guide_location, self.a_b, self.a_k, self.u_b, self.u_k, self.uav_count)
                else:
                    self.formation['prism'] = self.prism_gen(self.guide_location, self.a_k, self.u_b, self.u_k, self.uav_count)
                if self.pick_formation_id:
                    self.set_formation_id()
                    self.pick_formation_id = False
                self.target_position = self.formation[self.formation_type][self.formation_id]
                self.calculate_loop_movement()
                if not self.uav_msg['uav_guide']['gps_noise_flag']:
                    self.heading = self.uav_msg['active_uav']['heading'] % 360.0
                    self.move_to_target(self.target_position)
                else:
                    if not self.lock_heading:
                        self.heading = 270.0
                        self.lock_heading = True
                    self.gps_noise_move_to_target(self.target_position)
            else:
                self.operation_phase += 1
        elif self.operation_phase == 2:
            pass
        else:
            if self.uav_msg['uav_guide']['dispatch']:
                self.send_move_cmd(0, 0, self.uav_msg['uav_guide']['heading'], 100.0)
            else:
                self.operation_phase += 1
        self.calculate_loop_movement()

    def move_to_target(self, target_position):
        dist = util.dist(target_position, self.pose)
        target_angle = math.atan2(target_position[0]-self.pose[0], -(target_position[1]-self.pose[1]))
        target_angle = math.degrees(target_angle)
        self.speed[0] = 100.0
        self.speed[1] = 0.0
        if dist < 100.0:
            # iha yi yavaslat
            self.speed[0] = self.uav_msg['uav_guide']['speed']['x'] + dist * 0.2
        self.send_move_cmd(self.speed[0], self.speed[1], target_angle, target_position[2])

    def gps_noise_move_to_target(self, target_position):
        dist = [
            math.sqrt((target_position[0] - self.loop_location[0]) * (target_position[0] - self.loop_location[0])),
            math.sqrt((target_position[1] - self.loop_location[1]) * (target_position[1] - self.loop_location[1])),
        ]
        self.speed[0] = self.uav_msg['uav_guide']['speed']['x'] - 1.0 + dist[0] * 0.25
        self.speed[1] = self.uav_msg['uav_guide']['speed']['y'] - 1.0 + dist[0] * 0.25
        if target_position[0] - self.loop_location[0] < 0.0:
            self.direction[0] = -1
        else:
            self.direction[0] = 1
        if target_position[1] - self.loop_location[1] < 0.0:
            self.direction[1] = -1
        else:
            self.direction[1] = 1
        self.send_move_cmd(self.speed[0], self.speed[1], self.heading, target_position[2])
        

    def process_uav_msg(self):
        self.pose = [self.uav_msg['active_uav']['location'][0],
                     self.uav_msg['active_uav']['location'][1],
                     self.uav_msg['active_uav']['altitude'],
                     self.uav_msg['active_uav']['heading']]
        

    def get_formation_data(self):
        self.guide_location = [
            self.uav_msg['uav_guide']['location'][0],
            self.uav_msg['uav_guide']['location'][1],
            self.uav_msg['uav_guide']['altitude']
        ]
        if self.formation_type != self.uav_msg['uav_formation']['type']:
            self.pick_formation_id = True
        self.formation_type = self.uav_msg['uav_formation']['type']
        if self.formation_type == 'arrow':
            self.a_b = self.uav_msg['uav_formation']['a_b']
        self.a_k = self.uav_msg['uav_formation']['a_k']
        self.u_b = self.uav_msg['uav_formation']['u_b']
        self.u_k = self.uav_msg['uav_formation']['u_k']

    def set_formation_id(self):
        uav_position_list = []
        nearest = {'id': -1, 'dist': 0}
        prefix = 'uav_'
        for id in range(len(self.uav_msg['uav_link'])):
            #print id, self.uav_msg['uav_link'][id][prefix + str(id)]['location']
            uav_position_list.append([
                id,
                float(self.uav_msg['uav_link'][id][prefix + str(id)]['location'][0]),
                float(self.uav_msg['uav_link'][id][prefix + str(id)]['location'][1]),
                float(self.uav_msg['uav_link'][id][prefix + str(id)]['altitude'])
            ])
        cx = -1
        for node in self.formation[self.formation_type]:
            cx += 1
            nearest = {'dist': None, 'id': -1}
            pop_id = None
            for next_uav_id in range(len(uav_position_list)):
                next_location = [
                    uav_position_list[next_uav_id][1],
                    uav_position_list[next_uav_id][2],
                    uav_position_list[next_uav_id][3]
                ]
                d = -(util.dist(node, next_location))
                if nearest['dist'] < d:
                    nearest['dist'] = d
                    nearest['id'] =  uav_position_list[next_uav_id][0]
                    pop_id = int(next_uav_id)
            #self.formation_id[str(nearest['id'])] = uav_position_list.pop(pop_id)
            #print nearest['id'], self.uav_id, nearest['dist'], cx, self.formation_id
            if nearest['id'] == self.uav_id:
                self.formation_id = cx
            uav_position_list.pop(pop_id)

    def rotateUndTranslate(self, formation_array, angle, pivot):
        for i in range(len(formation_array)):
            sin_value = util.getSin(angle)
            cos_value = util.getCos(angle)
            p = formation_array[i]
            formation_array[i] = [
                (p[0] * cos_value - p[1] * sin_value) + pivot[0],
                (p[0] * sin_value + p[1] * cos_value) + pivot[1],
                p[2]
            ]
        return formation_array

    def arrow_gen(self, guide_location, a_b, a_k, u_b, u_k, uav_count):
        arrow_formation = []
        pivot = [guide_location[0], guide_location[1]]
        a_k = (a_k - 90.0) % 360.0
        x = 0.0 - u_k
        y = 0.0
        z = guide_location[2]
        arrow_formation.append([x, y, z])
        left_wing_angle = (180.0 - a_b) % 360.0
        left_sin_value = util.getSin(left_wing_angle)
        left_cos_value = util.getCos(left_wing_angle)
        right_wing_angle = (180.0 + a_b) % 360.0
        right_sin_value = util.getSin(right_wing_angle)
        right_cos_value = math.cos(math.radians(right_wing_angle))
        row_multiplier = 1
        next_is_left = True
        while(len(arrow_formation) < uav_count):
            if next_is_left:
                x = arrow_formation[0][0] + u_b * left_cos_value * row_multiplier
                y = arrow_formation[0][1] + u_b * left_sin_value * row_multiplier
            else:
                x = arrow_formation[0][0] + u_b * right_cos_value * row_multiplier
                y = arrow_formation[0][1] + u_b * right_sin_value * row_multiplier
                row_multiplier = row_multiplier + 1
            next_is_left = not next_is_left
            arrow_formation.append([x, y, z])
        #arrow_formation = self.rotateUndTranslate(arrow_formation, a_k, pivot)
        return self.rotateUndTranslate(arrow_formation, a_k, pivot)

    def prism_gen(self, guide_location, a_k, u_b, u_k, uav_count):
        prism_formation = []
        pivot = [guide_location[0], guide_location[1]]
        a_k = (a_k - 90.0) % 360.0
        x = 0.0 - u_k
        y = 0.0
        z = guide_location[2]
        prism_formation.append([x, y, z])
        row_multiplier = 1
        row_node = 1
        while(len(prism_formation) < uav_count):
            x = prism_formation[0][0] - u_b * row_multiplier
            if row_node % 2 == 1:
                y = prism_formation[0][1] + u_b / 2.0
            else:
                y = prism_formation[0][1] - u_b / 2.0
            if row_node < 3:
                z = prism_formation[0][2] + u_b / 2.0
            else:
                z = prism_formation[0][2] - u_b / 2.0
                if row_node == 4:
                    row_node = 0
                    row_multiplier = row_multiplier + 1
            prism_formation.append([x, y, z])
            row_node = row_node + 1
        #prism_formation = self.rotateUndTranslate(prism_formation, a_k, pivot)
        #return prism_formation
        return self.rotateUndTranslate(prism_formation, a_k, pivot)

    def calculate_loop_movement(self): 
        self.loop_time = self.uav_msg['sim_time'] - self.pre_sim_time
        self.pre_sim_time = self.uav_msg['sim_time']
        if self.uav_msg['uav_guide']['gps_noise_flag']:
            self.loop_location[0] += self.speed[0] * self.loop_time * self.k * self.direction[0]
            self.loop_location[1] += self.speed[1] * self.loop_time * self.k * self.direction[1]
        else:
            self.loop_location[0] = self.uav_msg['active_uav']['location'][0]
            self.loop_location[1] = self.uav_msg['active_uav']['location'][1]
        print self.loop_location, self.uav_msg['active_uav']['location'], self.speed, self.heading, self.uav_msg['uav_guide']['gps_noise_flag']
