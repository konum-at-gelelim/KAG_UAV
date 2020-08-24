from base.base_uav import BaseUAV
import math
import random
import util
import time


class KagUAV(BaseUAV):

    def initialize(self):
        self.fallback=False
        self.start_loc=None
        self.temp = 0
        self.pid_flag=False
            #
        self.target_position = None
        self.SIZE = 600
        self.COUNT = 10
        self.SPEED = 100
        self.FOLLOWERS = 4
        self.temp = 0
        self.LJP_EPSILON = 10 #iki atom arasi minimum uzaklik
        self.LJP_SIGMA = 3.3 #kuvvet birimi
        self.locdifftemp = 1942.27
        self.virtualgpsx = 0
        self.virtualgpsy = 0
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

        # k = 1 unit of [*_speed] /  1 uint of [sim_time]
        #change of location -> (speed * time) * k
        self.k = 0.0005144444
        self.gps_noise_loc=[0,0]

    def act(self):
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.amifallback()
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
            if self.uav_msg['uav_guide']['gps_noise_flag']==False:
                self.gps_noise_loc=[self.uav_msg['active_uav']['location'][0], self.uav_msg['active_uav']['location'][1]]
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
                if self.uav_msg['uav_guide']['gps_noise_flag']:
                    self.speed_calc()
                    self.gps_move(self.target_position)

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




    def gps_move(self,target):
        pass

    def speed_calc(self):
        if(self.temp >= 1):
            self.averagehead = ((self.uav_msg["active_uav"]['heading'] + self.oldheading)/2) % 360.0
            self.averagex = self.x + math.sin(math.radians(self.averagehead)) * self.uav_msg['active_uav']['x_speed']
            self.averagex = (self.averagex + math.sin(math.radians((self.averagehead - 90.0) % 360.0)) * self.uav_msg['active_uav']['y_speed'])/2
            self.averagey = self.y - math.cos(math.radians(self.averagehead)) * self.uav_msg['active_uav']['x_speed']
            self.averagey = (self.averagey - math.cos(math.radians((self.averagehead - 90.0) % 360.0)) * self.uav_msg['active_uav']['y_speed'])/2
            self.timediff = self.uav_msg['sim_time'] - self.time
            self.instantxdiff = self.averagex * self.timediff / self.locdifftemp # bi onceki paketle , simdiki paket arasi vakit ve hiz ortalamasi carpimi.
            self.instantydiff = self.averagey * self.timediff / self.locdifftemp
            self.headingdiff = self.uav_msg["active_uav"]['heading'] - self.oldheading
            print("x = ",self.uav_msg['active_uav']['x_speed']," y = ",self.uav_msg['active_uav']['y_speed']," head = ",self.uav_msg["active_uav"]['heading'])
            print("average x and y = ", self.averagex, self.averagey, "headingdiff = ", self.headingdiff)
            print("timediff = ",self.timediff)
            print("tahmini yer degisirme = ", self.instantxdiff, self.instantydiff)
        self.x = math.sin(math.radians(self.uav_msg["active_uav"]['heading'])) * self.uav_msg['uav_guide']['speed']['x']
        self.x = self.x + math.sin(math.radians((self.uav_msg["active_uav"]['heading'] - 90.0) % 360.0)) * self.uav_msg['uav_guide']['speed']['y']
        self.y = -(math.cos(math.radians(self.uav_msg["active_uav"]['heading'])) * self.uav_msg['uav_guide']['speed']['x'])
        self.y = self.y - math.cos(math.radians((self.uav_msg["active_uav"]['heading'] - 90.0) % 360.0)) * self.uav_msg['uav_guide']['speed']['y']
        if(self.temp == 0):
            #gps bozuklugu baslamadan once baslamali bu program.
            #gps bozuldugunda ilk alinan deger bozuk olacak..!!!

            self.virtualgpsx = self.gps_noise_loc[0]
            self.virtualgpsy = self.gps_noise_loc[1]

            self.timetemp = self.uav_msg['sim_time']
            self.instantxdiff = 0
            self.instantydiff = 0
        self.oldheading = self.uav_msg["active_uav"]['heading']
        self.time = self.uav_msg['sim_time']
        if(self.temp >= 1):
            xdiff = self.uav_msg['active_uav']['location'][0] - self.xloc
            ydiff = self.uav_msg['active_uav']['location'][1] - self.yloc
            if(self.instantxdiff != 0 and self.instantxdiff != 0):
                #print("locdiff / timediff * speed = ", xdiff / self.instantxdiff , ydiff / self.instantxdiff)
                print("real diff on x line = ", xdiff, "real diff of y line = ", ydiff)
                #print("timediff * speed / locdiff = ", self.instantxdiff / xdiff , self.instantydiff / ydiff)
            #self.instantxdiff = xdiff * self.speedcoeff
            #self.instantydiff = ydiff * self.speedcoeff
        self.xloc = self.uav_msg['active_uav']['location'][0]
        self.yloc = self.uav_msg['active_uav']['location'][1]
        if(self.temp >= 1):
            self.virtualgpsx = self.virtualgpsx + self.instantxdiff
            self.virtualgpsy = self.virtualgpsy + self.instantydiff
            print("tahmini gps = ",self.virtualgpsx,self.virtualgpsy)
            print("real gps = ",self.xloc,self.yloc)
            #print("tahmin edilen x deger = ",self.virtualgpsx, " + ",self.instantxdiff," = ", self.virtualgpsx + self.instantxdiff)
            #print("tahmin edilen y deger = ",self.virtualgpsy, " + ",self.instantydiff," = ", self.virtualgpsy + self.instantydiff)
        self.temp = self.temp + 1



    def move_to_target(self, target_position):
        dist = util.dist(target_position, self.pose)
        target_angle = math.atan2(target_position[0]-self.pose[0], -(target_position[1]-self.pose[1]))
        target_angle = math.degrees(target_angle)
        dist = util.dist(target_position, self.pose)
        x_speed = 90
        x_speed,y_speed=self.getXY(target_position[0],target_position[1],x_speed)
        self.send_move_cmd(x_speed,y_speed, target_angle, target_position[2])




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


    def amifallback(self):
        fuel=self.uav_msg['active_uav']["fuel_reserve"]
        if self.start_loc==None:
            pass
        if self.fallback==False:
            knot=20/3
            dist= util.dist(self.start_loc,[self.uav_msg['active_uav']['location'][0],self.uav_msg['active_uav']['location'][1]])
            #knot to kmh
            dist=dist*1.852
            #aradaki knot mesafe * knot basina harcanan yakit.
            fuel_=dist*knot

            if fuel>fuel_:
                pass
            if fuel<fuel_:
                self.fallback=True
        if self.fallback==True:
            pass



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

        print(error)
        self.Cp = error
        self.Ci += error * float(dt/100)
        self.Cd = de / dt
        self.Cd=self.Cd*100
        self.previous_time = current_time#ms
        self.previous_error = error
        print("turev",self.Kd * self.Cd)
        print("int",self.Ki * self.Ci)
        print("p :",self.Kp * self.Cp)
        file2=open("test15.txt",'a')
        data=file2.write(str(error))
        data=file2.write(" ")
        data=file2.write(str(self.uav_msg["sim_time"]))
        data=file2.write(" ; ")
        file2.close()
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
            dist= util.dist(self.start_loc,[self.uav_msg['active_uav']['location'][0],self.uav_msg['active_uav']['location'][1]])
            dist=dist/1.852 #knot to kmh
            fuel_=dist*knot #aradaki knot mesafe * knot basina harcanan yakit.
            fuel_=fuel_*2.2
            #print(fuel_,dist)
            if fuel>fuel_:
                pass
            if fuel<fuel_:
                self.fallback=True
        if self.fallback==True:
            pass
#####################################################################################################
# y = 0.24005X - 0.49546
    def getXY(self,x,y,speed):
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
        print(dist)
        self.PID(0.5,0.0,35.0)
        hm=self.Update(dist)
        print(hm)
        if hm>90:
            hm=90
        xx=xx*hm
        yy=yy*hm
        print("istenen:",xx,yy)
        return xx,yy

    def findAngle(self,x,y):
        fark=[0,0]
        uav_x=self.uav_msg["active_uav"]["location"][0]
        uav_y=self.uav_msg["active_uav"]["location"][1]
        fark[0]=x-uav_x
        # 90 derece farki icin -y
        fark[1]=uav_y-y
        aci=math.atan2(fark[0],fark[1])
        angle=math.degrees(aci)
        return angle
