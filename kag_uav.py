from base.base_uav import BaseUAV
import numpy as np
import math
import random
import util
import time


class KagUAV(BaseUAV):

    def initialize(self):
        self.fallback=False
        self.temp = 0
        self.pid_flag=False
        self.target_position = None
        self.LJP_EPSILON = 0.0103 #iki atom arasi minimum uzaklik
        self.LJP_SIGMA = 3.3 #kuvvet birimi
        self.locdifftemp = 1942.27
        self.egilmeFlag = 0
        self.yukselmeFlag = 0
        self.carpismaCemberi = 30
        #
        self.iteration_count = 0
        self.home = None
        self.start_loc = None
        self.fallback=False
        self.take_off = False
        self.last_state = None
        self.dispatch_is_okey = False
        self.formation_phase = None
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
        self.brake_limit = 100
        self.brake_timer = 100
        # k = 1 unit of [*_speed] /  1 uint of [sim_time]
        #change of location -> (speed * time) * k
        self.interrupt_loc = None
        self.injury_operation_phase = 1
        self.injury_load_phase = 0
        self.load_time = float(self.params['injured_pick_up_duration'])
        self.unload_time = float(self.params['injured_release_duration'])
        self.injury_timer = None
        self.time = None
        self.gps_heading = None
        self.gps_alt = None
        self.k = 0.0005144444
        self.gps_noise_loc=[0,0]
        self.x = None
        self.y = None
        self.oldheading = None
        self.averagehead = None
        self.xloc = None
        self.yloc = None
        self.alt_lock = None
        self.cruise_control = False

    def act(self):

        # umutun ve erenin codelari duzenlendi
        self.starting_func() # baslangic konum degerleri kayidi
        self.speed_calc() # gps bozuklugu durumu speed hesabi
        self.time_calc() # location_calc icin paket suresi guncellemesi
        self.location_calc() # gps bozuklugu durumu location hesabi
        self.process_uav_msg() # gps degerleri self.pose a aktarilir
        self.force_vector_calc() # carpismadan kacinma icin kuvvet hesabi
        self.col_avo() # kuvvetin iha dinamigine aktarimi
        self.brake_calc() # max speed hesabi
        print("self.operation_phase =", self.operation_phase)
        self.pre_formation()
        self.formation_func() # formasyon motoru
        self.amifallback() # geri donus karar verme araci
        self.move_to_home() # eve donus komutu

    def acc_calc(self, Speed_diff):
        print("timediff for speed =", self.timediff)
        #print("speed diff", Speed_diff[0], Speed_diff[1])
        if(Speed_diff[0] <= 5 and Speed_diff[0] >= -5):
            #print("xDiff = ", Speed_diff[0], "*", self.timediff, " = ", 0.00115148 * Speed_diff[0] * self.timediff)
            xDiff = (0.00115148 * Speed_diff[0]) * self.timediff
            if(Speed_diff[1] <= 5 and Speed_diff[1] >= -5):
                #print("yDiff = ", Speed_diff[1], "*", self.timediff, " = ", 0.00115148 * Speed_diff[1] * self.timediff)
                yDiff = (0.00115148 * Speed_diff[1]) * self.timediff
            elif(Speed_diff[1] > 5):
                yDiff = 0.0041 * self.timediff
                #print("yDiff = ", yDiff)
            else:
                yDiff = 0.0041 * self.timediff * -1
                #print("yDiff = ", yDiff)
        elif(Speed_diff[0] > 5):
            xDiff = yDiff = 0.0041 * self.timediff
            if(Speed_diff[1] <= 5 and Speed_diff[1] >= -5):
                #print("bum")
                yDiff = (0.00115148 * Speed_diff[1]) * self.timediff
            elif(Speed_diff[1] > 5):
                yDiff = 0.0041 * self.timediff
            else:
                yDiff = 0.0041 * self.timediff * -1
        else:
            xDiff = 0.0041 * self.timediff * -1
            if(Speed_diff[1] <= 5 and Speed_diff[1] >= -5):
                #print("bumyDiff = ", Speed_diff[1], "*", self.timediff, " = ", 0.00115148 * Speed_diff[1] * self.timediff)
                yDiff = (0.00115148 * Speed_diff[1]) * self.timediff
            elif(Speed_diff[1] > 5):
                yDiff = 0.0041 * self.timediff
                #print("yDiff = ", yDiff)
            else:
                yDiff = 0.0041 * self.timediff * -1
                #print("yDiff = ", yDiff)
        #print("xdiff, ydiff =", xDiff, yDiff)
        return xDiff, yDiff

    def speed_calc(self):
        if(self.uav_msg['uav_guide']['gps_noise_flag'] == True):
            Speed_diff = [self.target_speed[0] - self.current_speed[0], self.target_speed[1] - self.current_speed[1]]
            xAddition, yAddition = self.acc_calc(Speed_diff)
            print("tahmini acc =", xAddition, yAddition)
            self.current_speed = [self.current_speed[0] + xAddition, self.current_speed[1] + yAddition]
            print("tahmini x and y speed = ",self.current_speed[0], self.current_speed[1])
        else:
            self.current_speed = [self.uav_msg['active_uav']['x_speed'], self.uav_msg['active_uav']['y_speed']]
        print("real x and y speed = ", self.uav_msg['active_uav']['x_speed'], self.uav_msg['active_uav']['y_speed'])

    def ljp(self, r, epsilon, sigma):
        if(r == self.carpismaCemberi):
            return 0
        else:
            return 48 * epsilon * np.power(sigma, 12) / np.power(r-self.carpismaCemberi, 13) \
            - 24 * epsilon * np.power(sigma, 6) / np.power(r-self.carpismaCemberi, 7)

    def col_avo(self):
        colTempAngle = (self.uav_msg["active_uav"]['heading'] - self.collisionAngle - 90) % 360
        #print("colTempAngle = ", colTempAngle)
        self.yspeedaddition = -math.cos(math.radians(colTempAngle))*self.collisionMagnitude
        self.xspeedaddition = math.sin(math.radians(colTempAngle))*self.collisionMagnitude
        print("col avo speed = ", int(self.xspeedaddition), int(self.yspeedaddition))

    def altitude_controller(self):
        tempAngle = self.collisionAngle
        print("heading and colheading =", self.uav_msg["active_uav"]['heading'], self.collisionAngle)
        #iha cok hizliyken kontrollu gerceklesmeli
        if(tempAngle <= (self.uav_msg["active_uav"]['heading']+2)%360 and tempAngle >= (self.uav_msg["active_uav"]['heading']-2)%360):
            #print("deadlock_error")
            if((self.uav_msg["active_uav"]['heading'])%360 > 0 and (self.uav_msg["active_uav"]['heading'])%360 < 180):
                #print("iha egiliyor")
                self.egilmeFlag = self.egilmeFlag + 1
            elif((self.uav_msg["active_uav"]['heading'])%360 > 180 and (self.uav_msg["active_uav"]['heading'])%360 < 360):
                #print("iha yukseliyor")
                self.yukselmeFlag = self.egilmeFlag + 1

    def brake_calc(self):
        brake_temp = 0
        brake_var = 0, 0
        distancex = 0
        distancey = 0
        for i in range(len(self.uav_msg['uav_link'])):
            a = self.uav_msg["uav_link"][i].keys()
            uav_name = a[0][4]
            uav_name = str(uav_name)
            if uav_name != str(self.uav_id):
                #i numarali ihanin speed izdusumleri.
                tempx = math.sin(math.radians(self.uav_msg["uav_link"][i].values()[0]["heading"])) * self.uav_msg["uav_link"][i].values()[0]["speed"]['x']
                tempx = tempx + math.sin((math.radians((self.uav_msg["uav_link"][i].values()[0]["heading"] - 90.0) % 360.0))) * self.uav_msg["uav_link"][i].values()[0]["speed"]['y']
                tempy = -math.cos(math.radians(self.uav_msg["uav_link"][i].values()[0]["heading"])) * self.uav_msg["uav_link"][i].values()[0]["speed"]['x']
                tempy = tempy - math.cos((math.radians((self.uav_msg["uav_link"][i].values()[0]["heading"] - 90.0) % 360.0))) * self.uav_msg["uav_link"][i].values()[0]["speed"]['y']
                #birbirlerine yaklasma degerleri
                #print("tempx - self.x =", tempx, self.x)
                #print("xspeed  =", self.uav_msg["uav_link"][i].values()[0]["speed"]['x'], self.uav_msg['active_uav']['x_speed'])
                tempx = tempx - self.x
                tempy = tempy - self.y
                #x ve y degerin magnitude'u
                magnOfSpeed = math.sqrt((tempx)**2+(tempy)**2)
                #aralarindaki mesafe
                distance = math.sqrt((self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])**2 + (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])**2)
                danger_calc = distance - (magnOfSpeed * 5.6)#5.6
                if(danger_calc < brake_temp):
                    distancex, distancey = (self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])/distance, (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])/distance
                    brake_temp = danger_calc
                    brake_var = distance, magnOfSpeed
                else:
                    continue
                #print("warning for =", i, "th IHA distance and speed = ", distance, magnOfSpeed)
        distance, magnOfSpeed = brake_var
        self.maxSpeed = abs(brake_temp / 5.6)
        self.maxSpeed = 90 - self.maxSpeed
        print("max speed =", self.maxSpeed)
        print("real speed =", self.uav_msg['active_uav']['x_speed'], self.uav_msg['active_uav']['y_speed'])
        self.brakeMagnitude = magnOfSpeed - self.maxSpeed
        self.brakeAngle = math.degrees(math.atan2(distancex, -distancey))
        #print("total brake(angle, magnitude) =", self.brakeAngle, self.brakeMagnitude)

    def force_vector_calc(self):
        ux = 0
        uy = 0
        for i in range(len(self.uav_msg['uav_link'])):
            a=self.uav_msg["uav_link"][i].keys()
            uav_name=a[0][4]
            uav_name=str(uav_name)
            if uav_name != str(self.uav_id) and self.pose[3] :
                distance = math.sqrt((self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])**2 + (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])**2)
                #print(i, "th IHA distance = ", distance)
                distancex, distancey = (self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])/distance, (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])/distance
                u = self.ljp(distance, self.LJP_EPSILON, self.LJP_SIGMA)
                ux = ux + u*distancex
                uy = uy + u*distancey
                #print(i, "th IHA forces = ", int(u*distancex), int(u*distancey))
        self.collisionAngle = math.atan2(ux, -uy)
        self.collisionAngle = math.degrees(self.collisionAngle)
        self.collisionMagnitude = math.hypot(ux,uy)
        #print("avoidance force = " , ux , " , " , uy)
        #print("total vector(angle, magnitude) = ", self.collisionAngle, self.collisionMagnitude)

    def time_calc(self):
        if(self.temp >= 1):
            self.timediff = self.uav_msg['sim_time'] - self.time
        else:
            self.time = self.uav_msg['sim_time']

    def location_calc(self):
        if(self.uav_msg['uav_guide']['gps_noise_flag'] == True):
            self.averagehead = ((self.uav_msg["active_uav"]['heading'] + self.oldheading)/2) % 360.0
            self.averagex = self.x + math.sin(math.radians(float(self.averagehead))) * self.current_speed[0]
            self.averagex = (self.averagex + math.sin(math.radians((self.averagehead - 90.0) % 360.0)) * self.current_speed[1])/2
            self.averagey = self.y - math.cos(math.radians(self.averagehead)) * self.current_speed[0]
            self.averagey = (self.averagey - math.cos(math.radians((self.averagehead - 90.0) % 360.0)) * self.current_speed[1])/2
            self.timediff = self.uav_msg['sim_time'] - self.time
            self.instantxdiff = self.averagex * self.timediff / self.locdifftemp # bi onceki paketle , simdiki paket arasi vakit ve hiz ortalamasi carpimi.
            self.instantydiff = self.averagey * self.timediff / self.locdifftemp
            #self.headingdiff = self.uav_msg["active_uav"]['heading'] - self.oldheading
            self.realgps = [self.realgps[0] + self.instantxdiff, self.realgps[1] + self.instantydiff]
            print("tahmini gps = ",self.realgps[0], self.realgps[1])
            print("real gps = ", self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1])
            print("average x and y = ", self.averagex, self.averagey)
            print("timediff = ", self.timediff)
            print("tahmini yer degisirme = ", self.instantxdiff, self.instantydiff)
        self.x = math.sin(math.radians(self.uav_msg["active_uav"]['heading'])) * self.current_speed[0]
        self.x = self.x + math.sin((math.radians((self.uav_msg["active_uav"]['heading'] - 90.0) % 360.0))) * self.current_speed[1]
        self.y = -math.cos(math.radians(self.uav_msg["active_uav"]['heading'])) * self.current_speed[0]
        self.y = self.y - math.cos((math.radians((self.uav_msg["active_uav"]['heading'] - 90.0) % 360.0))) * self.current_speed[1]
        self.oldheading = self.uav_msg["active_uav"]['heading']
        self.time = self.uav_msg['sim_time']
        if(self.uav_msg['uav_guide']['gps_noise_flag'] == False):
            self.realgps = [self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1]]
        self.temp = self.temp + 1

    def starting_func(self):
        if self.home == None:#eve donus degiskeni
            self.home = (self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1], 100.0)#eve donmek icin baslangic konumlari
            self.start_loc = [self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1]]
            self.pre_location = [self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1]]
            self.uav_id = int(self.uav_id)
            self.pre_sim_time = self.uav_msg['sim_time']
            self.heading = self.uav_msg['active_uav']['heading'] % 360.0

    def move_to_home(self):
        if self.operation_phase == 3:# 0-> kalkis 1->formasyon sureci 2->gorevler 3->eve donus
            self.move_to_target(self.home)

    def formation_func(self):
        if self.operation_phase == 1:
            if not self.uav_msg['uav_guide']['dispatch']:
                self.formation_setup()
                self.gps_alt = self.pose[2]
                self.move_to_target(self.target_position)
            else:
                self.operation_phase = self.operation_phase + 2 # formasyon bittiyse sonraki adima gecis

    def pre_formation(self):
        if self.operation_phase == 0:
            if self.uav_msg['uav_guide']['dispatch']:
                self.formation_setup()
                self.move_to_target(self.target_position)
            else:
                self.operation_phase += 1

    def bum_func(self):
        if self.operation_phase == 2:
            if self.injury_operation_phase:
                A = [198.297, 77.702, float(self.params['injured_pick_up_height'])]
                B = [125.0, -440.0, float(self.params['injured_release_height'])]
                self.injury_operation(A, B)
                print(self.injury_operation_phase, self.injury_load_phase)

    def injury_operation(self, injured_xy, hospital_xy):
        if self.interrupt_loc == None:
            self.interrupt_loc = [self.pose[0], self.pose[1], self.pose[2]]
        if self.injury_operation_phase == 1:
            self.injury_load_process(injured_xy, 'load')
        elif self.injury_operation_phase == 2:
            self.injury_load_process(hospital_xy, 'unload')
        else:
            self.injury_operation_phase = 0
            print('hoooraaaaa! We saved mother russia!')
            pass

    def injury_load_process(self, target, load_type):
        if self.injury_load_phase == 0:
            d = util.dist(self.pose[:2], target[:2])
            if not self.reached(d):
                self.move_to_target((target[:2] + [90.0]))
            else:
                self.injury_load_phase += 1
        elif self.injury_load_phase == 1:
            print(self.alt_lock == self.pose[2], self.alt_lock, self.pose[2])
            if self.pose[2] < (target[2] - 2.0):
                print('a')
                if self.alt_lock == None:
                    print('b')
                    self.alt_lock = float((target[2] - 2.0))
                elif self.is_load_done(load_type):
                    print('d')
                    self.injury_load_phase += 1
                print('z')
                self.target_speed = [0.0,0.0]
                self.send_move_cmd(0.0, 0.0, self.pose[3], self.alt_lock)
            else:
                print('x')
                self.target_speed = [0.0,0.0]
                self.send_move_cmd(0.0, 0.0, self.pose[3], 2.5)
        elif self.injury_load_phase == 2:
            if self.pose[2] < 90.0:
                self.target_speed = [0.0,0.0]
                self.send_move_cmd(0, 0, self.pose[3], 100.0)
            else:
                self.target_speed = [0.0,0.0]
                self.send_move_cmd(0, 0, self.pose[3], self.pose[2])
                self.injury_load_phase = 0
                self.injury_operation_phase += 1


    def is_load_done(self, load_type):
    	if self.injury_timer == None:
    		self.injury_timer = self.uav_msg['sim_time']
        if load_type == 'load':
            if self.uav_msg['sim_time'] - self.injury_timer > (self.load_time * 1000):
                self.injury_timer = None
                return True
            else:
            	print((self.uav_msg['sim_time'] - self.injury_timer), self.alt_lock, self.pose[2], self.alt_lock == self.pose[2])
                return False
        else:
            if self.uav_msg['sim_time'] - self.injury_timer > (self.load_time * 1000):
                self.injury_timer = None
                return True
            else:
            	print((self.uav_msg['sim_time'] - self.injury_timer), self.alt_lock, self.pose[2], self.alt_lock == self.pose[2])
                return False

    def formation_setup(self):
        self.get_formation_data()
        if self.formation_type == 'arrow':
            self.formation['arrow'] = self.arrow_gen(self.guide_location, self.a_b, self.a_k, self.u_b, self.u_k, self.uav_count)
        else:
            self.formation['prism'] = self.prism_gen(self.guide_location, self.a_k, self.u_b, self.u_k, self.uav_count)
        if self.pick_formation_id:
            self.set_formation_id()
            self.pick_formation_id = False
        self.target_position = self.formation[self.formation_type][self.formation_id]
        print(self.formation[self.formation_type])

    def formation_move(self, target_position):
        dist = util.dist(target_position, self.pose)
        target_angle = math.atan2(target_position[0]-self.pose[0], -(target_position[1]-self.pose[1]))
        target_angle = math.degrees(target_angle) % 360.0

        x_speed = self.uav_msg['uav_guide']['speed']['x']
        if self.brake_timer < self.brake_limit:
            self.brake_timer += 1
            target_angle = self.pose[3]
            x_speed = 0
        else:
            if not self.reached(dist):
                x_speed += dist * 0.125
        if target_position[2] < 1.0:
            target_position[2] = 1.0
        self.target_speed = [x_speed * 1.00133, 0.0]
        self.send_move_cmd(x_speed, 0.0, target_angle, target_position[2])

    def move_to_target(self, target_position):
        dist = util.dist(target_position, self.pose)
        target_angle = math.atan2(target_position[0]-self.pose[0], -(target_position[1]-self.pose[1]))
        target_angle = math.degrees(target_angle)

        x_speed, y_speed=self.getXY(target_position[0], target_position[1], self.maxSpeed)
        x_speed = self.xspeedaddition + x_speed
        y_speed = self.yspeedaddition + y_speed
        if target_position[2] < 1.0:
            target_position[2] = 1.0
        if dist > 30.0:
        	x_speed = 20.0
        self.target_speed = [x_speed *  1.00133, y_speed *  1.00133]
        self.send_move_cmd(x_speed,y_speed, target_angle, target_position[2])

    def reached(self, dist):
        if dist < 3:
            return True
        else:
            return False

    def process_uav_msg(self):
        self.pose = [self.realgps[0],
                     self.realgps[1],
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

    def PID(self, Kp, Ki, Kd, origin_time=None):
        if self.pid_flag:
            return 0
        if origin_time is None:
            origin_time = self.uav_msg["sim_time"]

        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_time = origin_time#ms
        self.previous_error = 0.0
        self.pid_flag=True

    def Update(self, error,current_time=None):
        if current_time is None:
            current_time = self.uav_msg["sim_time"]#ms
        dt = current_time - self.previous_time #ms
        if dt <= 0.0:
            return 0
        de = error - self.previous_error

        #print(error)
        self.Cp = error
        self.Ci += error * float(dt/100)
        self.Cd = de / dt
        self.Cd=self.Cd*100
        self.previous_time = current_time#ms
        self.previous_error = error
        #print("turev",self.Kd * self.Cd)
        #print("int",self.Ki * self.Ci)
        #print("p :",self.Kp * self.Cp)
        return (
            (self.Kp * self.Cp)    # proportional term
            + (self.Ki * self.Ci)  # integral term
            + (self.Kd * self.Cd)  # derivative term
        )
##################################saha ici hesaplanmadi #############################################
    def amifallback(self):
        fuel=self.uav_msg['active_uav']["fuel_reserve"]
        if self.start_loc==None:
            pass
        if self.fallback==False:
            knot=0.0036
            dist= util.dist(self.start_loc, [self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1]])
            dist=dist/1.852 #knot to kmh
            fuel_=dist*knot #aradaki knot mesafe * knot basina harcanan yakit.
            fuel_=fuel_*2.2
            #print(fuel_,dist)
            if fuel>fuel_:
                pass
            if fuel<fuel_:
                self.fallback=True
                self.operation_phase=3
        if self.fallback==True:
            pass

#####################################################################################################
# y = 0.24005X - 0.49546
    def getXY(self, x, y, speed):
        target_position=[x,y]
        head=self.uav_msg["active_uav"]["heading"]
        targetAngle=self.findAngle(x,y)
        if targetAngle <0:
            targetAngle=360+targetAngle
        head=head-targetAngle
        #target_position=[x,y]
        #dist = util.dist(target_position, self.pose)
        head=math.radians(head)
        yy=math.sin(head)
        xx=math.cos(head)
        dist = util.dist(target_position, self.pose)
        #print(dist)
        self.PID(0.5,0.0,35.0)
        hm=self.Update(dist)
        #print(hm)
        if hm>speed:
            hm=speed
        xx=xx*hm
        yy=yy*hm
        #print("istenen:",xx,yy)
        return xx,yy

    def findAngle(self,x,y):
        fark=[0,0]
        uav_x=self.pose[0]
        uav_y=self.pose[1]
        fark[0]=x-uav_x
        # 90 derece farki icin -y
        fark[1]=uav_y-y
        aci=math.atan2(fark[0],fark[1])
        angle=math.degrees(aci)
        return angle
