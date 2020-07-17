from base.base_uav import BaseUAV
import math
import random
import util
import time


class KekUAV(BaseUAV):

    def initialize(self):
        self.iteration_count = 0
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
        self.formation_node = {}
        self.pick_formation_node = True
        self.formation = {'arrow': [], 'prism': []}
        self.axisMagMax = 50.0
        self.pushForceRadius = 50.0
        self.VS = []

    def act(self):
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        # if self.uav_msg['uav_guide']['dispatch']:
        #     print('uav_guide.dispath is True')
        
        self.get_formation_data()
        if self.formation_type == 'arrow':
            self.formation['arrow'] = self.arrow_gen(self.guide_location, self.a_b, self.a_k, self.u_b, self.u_k, self.uav_count)
        else:
            self.formation['prism'] = self.prism_gen(self.guide_location, self.a_k, self.u_b, self.u_k, self.uav_count)
            
        self.set_formation_node()
        self.target_position = util.Point_3d(
            self.formation_node[str(self.uav_id)][1],
            self.formation_node[str(self.uav_id)][2],
            self.formation_node[str(self.uav_id)][3]).getList()
        self.set_target_position()
        print self.formation_node[str(self.uav_id)], self.target_position
        print self.uav_msg['sim_time']
        for i in self.VS:
            print i
        print "\n"

        #print 10 * '-', 'arrow', self.uav_msg['sim_time'], 10 * '-'
        #for i in self.formation['arrow']:
        #    print i
        #print ""
        #print 10 * '-', 'prism', self.uav_msg['sim_time'], 10 * '-'
        #for i in self.formation['prism']:
        #    print i
        #print ""
        #self.target_position = self.formation['prism'][int(self.uav_id)]
        #self.move_to_target(self.target_position)
        #print self.uav_msg['active_uav']['x_speed'], self.uav_msg['uav_link'][5]['uav_5']

    def reach_to_target(self):
        if self.target_position is None:
            return True

        thresh = 3.0 # bu degerden daha yakinsak vardik sayiyoruz
        dist = util.dist(self.target_position, self.pose)
        if dist < thresh:
            return True
        else:
            return False

    def move_to_target(self, target_position):
        dist = util.dist(target_position, self.pose)
        # print('random target position x:%f y:%f altitude:%f' %
        #       (target_position[0], target_position[1], target_position[2]))
        # print('error:' + str(dist))
        target_angle = math.atan2(target_position[0]-self.pose[0], -(target_position[1]-self.pose[1]))
        target_angle = math.degrees(target_angle)
        dist = util.dist(target_position, self.pose)
        x_speed = 100.0
        if dist < 200.0:
            # iha yi yavaslat
            x_speed = dist*0.50
        self.send_move_cmd(x_speed, 0, target_angle, target_position[2])

    def process_uav_msg(self):
        self.pose = [self.uav_msg['active_uav']['location'][0],
                     self.uav_msg['active_uav']['location'][1],
                     self.uav_msg['active_uav']['altitude'],
                     self.uav_msg['active_uav']['heading']]
        
    def set_target_position(self, target=None):
        if target is None:
            return
        else:
            origin = util.Point_3d(self.pose[0], self.pose[1], self.pose[2])
            base_vector = (util.point_3_vector(self.target_position - origin)).setFera(self.axisMagMax)
            prefix = 'uav_'
            for id in range(len(self.uav_msg['uav_link'])):
                if id != self.uav_id:
                    next_uav = util.Point_3d(
                        self.uav_msg['uav_link'][id][prefix + str(id)]['location'][0],
                        self.uav_msg['uav_link'][id][prefix + str(id)]['location'][1],
                        self.uav_msg['uav_link'][id][prefix + str(id)]['altitude']
                    )
                d = (util.point_3_vector(origin - next_uav)).getMag()
                if d <= self.pushForceRadius:
                    base_vector += util.Vector_3d(
                        -(d.x) % self.pushForceRadius,
                        -(d.y) % self.pushForceRadius,
                        -(d.z) % self.pushForceRadius,
                    )
                    self.VS.append([id, base_vector])
            self.target_position = (
                self.target_position[0] + base_vector.x,
                self.target_position[1] + base_vector.y,
                self.target_position[2] + base_vector.z,
            )
        

    def get_formation_data(self):
        self.guide_location = [
            self.uav_msg['uav_guide']['location'][0],
            self.uav_msg['uav_guide']['location'][1],
            self.uav_msg['uav_guide']['altitude']
        ]
        if self.formation_type != self.uav_msg['uav_formation']['type']:
            self.pick_formation_node = True
        self.formation_type = self.uav_msg['uav_formation']['type']
        if self.formation_type == 'arrow':
            self.a_b = self.uav_msg['uav_formation']['a_b']
        self.a_k = self.uav_msg['uav_formation']['a_k']
        self.u_b = self.uav_msg['uav_formation']['u_b']
        self.u_k = self.uav_msg['uav_formation']['u_k']

    def set_formation_node(self):
        uav_position_list = []
        nearest = {'id': -1, 'dist': 0}
        prefix = 'uav_'
        for id in range(len(self.uav_msg['uav_link'])):
            #print id, self.uav_msg['uav_link'][id][prefix + str(id)]['location']
            uav_position_list.append([
                id,
                self.uav_msg['uav_link'][id][prefix + str(id)]['location'][0],
                self.uav_msg['uav_link'][id][prefix + str(id)]['location'][1],
                self.uav_msg['uav_link'][id][prefix + str(id)]['altitude']
            ])
        cx = 0
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
                    pop_id = next_uav_id
            #self.formation_node[str(nearest['id'])] = uav_position_list.pop(pop_id)
            temp_var = uav_position_list.pop(pop_id)
            temp_var[0] = cx
            self.formation_node[str(nearest['id'])] = temp_var

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

