from base.base_uav import BaseUAV
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from scipy.spatial import ConvexHull
import matplotlib.path as mpltPath
from scipy import stats
from numpy import arange
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
        self.defter = []# uav_link gps bozuklugu durumunda regresyon ile konum guncelleyecek aksi durumda ayni veriler devam edecek
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
        self.preheading = 380 # -360 ve +360 disinda bir deger koyuldu
        
        self.deniedZones = self.params['denied_zones']
        self.world_boundaries = self.params['world_boundaries']
        #self.casualties = self.uav_msg['casualties_in_world']
        self.uav_communication_range = self.params['uav_communication_range']
        self.telecom_radius = self.params['telecom_radius']
        self.world_width = float(self.params['world_width'])
        self.idle_parkour = None
        self.world_length = float(self.params['world_length'])
        self.the_n = 3
        
    def act(self):
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        self.engine_method()

    def telecom_distance_sort_aux_method(self, e):
        return e['distance']

    def telecom_second_distance_sort_aux_method(self, e):
        return e['d']
    
    def telecom_cluster_sort_method(self, e):
        return e['cluster']
    
    def uav_injured_center_dist_sort_method(self, e):
        return e['injured_center_dist']
    
    def uav_id_sort_method(self, e):
        return e['id']
    
    def idle_parkour_gen(self):
        xmin = self.world_boundaries[0][0]
        ymin = self.world_boundaries[0][1]
        xmax = self.world_boundaries[0][0]
        ymax = self.world_boundaries[0][1]
        result_list = []
        for i in range(1, len(self.world_boundaries)):
            if self.world_boundaries[i][0] < xmin:
                xmin = self.world_boundaries[i][0]
            elif self.world_boundaries[i][0] > xmax:
                xmax = self.world_boundaries[i][0]
            if self.world_boundaries[i][1] < ymin:
                ymin = self.world_boundaries[i][1]
            elif self.world_boundaries[i][1] > ymax:
                ymax = self.world_boundaries[i][1]
        world_center = [(xmax + xmin) / 2.0, (ymax + ymin) / 2.0]
        for i in range(len(self.world_boundaries)):
            temp_point = [None, None]
            temp_point[0] = (self.world_boundaries[i][0] - world_center[0]) / 2.0
            temp_point[1] = (self.world_boundaries[i][1] - world_center[1]) / 2.0
            if self.notInDeniedZone([temp_point], self.deniedZones):
                result_list.append(temp_point)
        if len(result_list) != 0:
            return result_list
        elif self.notInDeniedZone([world_center], self.deniedZones):
            return [world_center, self.pose[:2]]
        else:
            return self.world_boundaries

    def printAll(self, listo, ayrac):
        print('*', str(ayrac), '#')
        for kekw in listo:
            print(kekw)
    
    def parkour_gen(self):
        self.idle_parkour = self.idle_parkour_gen()
        self.printAll(self.idle_parkour, 'idle park')
    
    def engine_method(self):
        DEFAULT = 'D'
        YOK = 'YOK'
        TELEKOM = 'T'
        KURTARMA = 'K'
        telecom_center = [None, None]
        injured_center = [None, None]
        cluster_count = 0
        telecom_cluster_centers = []
        telecom_nodes = []
        injured_nodes = []
        telecom_task_possible = False
        injured_task_possible = False
        if len(self.uav_msg['casualties_in_world']) != 0:
            for i in range(len(self.uav_msg['casualties_in_world'])):
                if self.uav_msg['casualties_in_world'][i]['status'] == 'healthy':
                    telecom_center[0] += self.uav_msg['casualties_in_world'][i]['pose'][0]
                    telecom_center[1] += self.uav_msg['casualties_in_world'][i]['pose'][1]
                    telecom_nodes.append({'pose': tuple(self.uav_msg['casualties_in_world'][i]['pose']), 'distance': None, 'cluster': -1})
                else:
                    injured_center[0] += self.uav_msg['casualties_in_world'][i]['pose'][0]
                    injured_center[1] += self.uav_msg['casualties_in_world'][i]['pose'][1]
                    injured_nodes.append({'pose': tuple(self.uav_msg['casualties_in_world'][i]['pose']), 'assigned_uav_id': None})    
            if len(telecom_nodes) != 0:
                telecom_center = [telecom_center[0] / len(telecom_nodes), telecom_center[1] / len(telecom_nodes)]
                telecom_task_possible = True
            if len(injured_nodes) != 0:
                injured_center = [injured_center[0] / len(injured_nodes), injured_center[1] / len(injured_nodes)]
                injured_task_possible = True
            self.printAll(telecom_center, 'telkom_mermez')
            self.printAll(injured_center, 'injur merkez')
            uav_list = []
            for next_uav_index in range(len(self.uav_msg['uav_link'])):
                for prefix_id in self.uav_msg['uav_link'][next_uav_index]:
                    uav_list.append({
                        'id': int(prefix_id[4:]),
                        'props': self.uav_msg['uav_link'][next_uav_index][prefix_id],
                        'assign': YOK,
                        'injured_center_dist': None
                    })
            self.printAll(uav_list, 'uav listo')
            
            ####KURTARMA
            if injured_task_possible:
                for i in range(len(uav_list)):
                    if not uav_list[i]['props']['injured_rescue_status']['isCarrying'] and uav_list[i]['props']['task'] == TELEKOM:
                        uav_list[i]['injured_center_dist'] = util.dist(injured_center, uav_list[i]['props']['location'])
                uav_list.sort(reverse=True, key=self.uav_injured_center_dist_sort_method)
                self.printAll(uav_list, 'injur mesafe')
                for i in range(len(uav_list)):
                    if uav_list[i]['injured_center_dist'] == None:
                        break
                    nearest = {'index': None, 'dist': None}
                    for j in range(len(injured_nodes)):
                        if injured_nodes[j]['assigned_uav_id'] != None:
                            continue
                        d = util.dist(uav_list[i]['props']['location'], injured_nodes[j]['pose'])
                        if (d < nearest['dist'] or nearest['dist'] == None):
                            nearest['dist'] = d
                            nearest['index'] = j
                    if nearest['index'] != None:
                        injured_nodes[nearest['index']]['assigned_uav_id'] = uav_list[i]['id']
                        uav_list[i]['assign'] = KURTARMA + '_' + str(nearest['index'])
                        uav_list[i]['props']['task'] = KURTARMA
                #uav_list.sort(key=self.uav_id_sort_method)
                for i in uav_list:
                    if i['assign'][0] == KURTARMA and i['id'] == self.uav_id:
                        #GÖREVVmove_to_target(injured_nodes[int(uav_list[uav_id]['assign'][2:])], uav_list[uav_id]['props']['task'])
                        print('BITTI BURADA METHOD, RETURN YAZIN BU PRINT YERINE!')
                for i in uav_list:
                    print(i['assign'])
                print('KEKW')
            ####TELEKOMMM
            if telecom_task_possible:
                for i in range(len(telecom_nodes)):
                    telecom_nodes[i]['distance'] = util.dist(telecom_center, telecom_nodes[i]['pose'])
                telecom_nodes.sort(reverse=True, key=self.telecom_distance_sort_aux_method)
                self.printAll(telecom_nodes, 'sort dist desc tip after effect')
                for i in range(len(telecom_nodes)):
                    if telecom_nodes[i]['cluster'] != -1:
                        continue
                    neighbour_list = [{'d': 0.0, 'tni': i}]
                    for j in range(len(telecom_nodes)):
                        if(i != j) and (telecom_nodes[j]['cluster'] == -1):
                            d = util.dist(telecom_nodes[i]['pose'], telecom_nodes[j]['pose'])
                            if d < self.telecom_radius:
                                neighbour_list.append({'d': d, 'tni': j})

                    if len(neighbour_list) >= the_n:
                        cluster_count += 1
                        neighbour_list.sort(key=self.telecom_second_distance_sort_aux_method)
                        cluster_center_temp = [0.0, 0.0]
                        
                        
                        for node in (neighbour_list[:self.the_n]):
                            telecom_nodes[node['tni']]['cluster'] = cluster_count-1
                            cluster_center_temp[0] += telecom_nodes[node['tni']]['pose'][0]
                            cluster_center_temp[1] += telecom_nodes[node['tni']]['pose'][1]
                        cluster_center_temp[0] /= self.the_n
                        cluster_center_temp[1] /= self.the_n
                        telecom_cluster_centers.append(cluster_center_temp)
                        for j in range(len(neighbour_list[:self.the_n])):
                            d = dist(telecom_cluster_centers[cluster_count-1], telecom_nodes[neighbour_list[j]['tni']]['pose'])
                            if d >= telecom_radius:
                                telecom_nodes[neighbour_list[j][tni]]['cluster'] = -1              
                
                telecom_nodes.sort(key=self.telecom_cluster_sort_method)
                self.printAll(telecom_cluster_centers, 'cluster centeros')
                self.printAll(telecom_nodes, 'telecom cluseter sort iste')
                for i in range(len(telecom_cluster_centers)):
                    nearest = {'index': None, 'dist': None}  
                    for j in range(len(uav_list)):
                        if uav_list[j]['assign'] != YOK:
                            continue
                        d = util.dist(telecom_cluster_centers[i], uav_list[j]['props']['location'])
                        if (d < nearest['dist'] or nearest['dist'] == None) and (uav_list[j]['props']['task'] == TELEKOM):
                            nearest['dist'] = d
                            nearest['index'] = j
                    if nearest['index'] != None:
                        uav_list[nearest['index']]['assign'] = TELEKOM + '_' + str(i)
                self.printAll(uav_list, 'esaynlar falan')
                for i in uav_list:
                    if i['assign'][0] == TELEKOM and i['id'] == self.uav_id:
                        #GÖREVVmove_to_target(injured_nodes[int(uav_list[uav_id]['assign'][2:])], uav_list[uav_id]['props']['task'])
                        print('UAV TELEKOMA ATANDI!!!!, burada method kendisi icin bitmeli!')
                if uav_list[uav_id]['assign'][0] == TELEKOM:
                    print(int(uav_list[uav_id]['assign'][2:]))
                    #move_to_target(telecom_cluster_centers[int(uav_list[uav_id]['assign'][2:])], uav_list[uav_id]['props']['task'])
            #do parkour!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!buraeklecenonuumutasor

    def speed_calc(self):
        if(self.uav_msg['uav_guide']['gps_noise_flag'] == True):
            Speed_diff = [self.target_speed[0] - self.current_speed[0], self.target_speed[1] - self.current_speed[1]]
            xAddition, yAddition = self.acc_calc(Speed_diff)
            print("tahmini acc =", float(xAddition), float(yAddition))
            self.current_speed = [self.current_speed[0] + xAddition, self.current_speed[1] + yAddition]
            print("tahmini x and y speed = ", int(self.current_speed[0]), int(self.current_speed[1]))
        else:
            self.current_speed = [self.uav_msg['active_uav']['x_speed'], self.uav_msg['active_uav']['y_speed']]
        print("real x and y speed = ", self.uav_msg['active_uav']['x_speed'], self.uav_msg['active_uav']['y_speed'])

    def ljp(self, r, epsilon, sigma):
        if(r <= self.uav_msg['uav_formation']['u_b'] - 2):
            return 824604576
        else:
            return 48 * epsilon * np.power(sigma, 12) / np.power(r - (self.uav_msg['uav_formation']['u_b'] - 2), 13) \
            - 24 * epsilon * np.power(sigma, 6) / np.power(r - (self.uav_msg['uav_formation']['u_b'] - 2), 7)

    def col_avo(self):
        colTempAngle = (self.uav_msg["active_uav"]['heading'] - self.collisionAngle - 90) % 360
        #print("colTempAngle = ", colTempAngle)
        self.yspeedaddition = -math.cos(math.radians(colTempAngle))*self.collisionMagnitude
        self.xspeedaddition = math.sin(math.radians(colTempAngle))*self.collisionMagnitude
        print("col avo speed = ", int(self.xspeedaddition), int(self.yspeedaddition))

    def altitude_controller(self):
        tempAngle = self.collisionAngle
        #print("heading and colheading =", self.uav_msg["active_uav"]['heading'], self.collisionAngle)
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
        brake_var = 0
        for i in range(len(self.uav_msg['uav_link'])):
            uav_name = self.uav_msg["uav_link"][i].keys()
            uav_name = uav_name[0][4:]
            if int(uav_name) != self.uav_id:
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
                gg = 3.121 * np.power(distance, 0.5342) + 4.214
                #danger_calc = distance - (magnOfSpeed * 5.6)
                danger_calc = gg - magnOfSpeed
                if(danger_calc > brake_temp):
                    #distancex, distancey = (self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])/distance, (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])/distance
                    brake_temp = danger_calc
                    brake_var = gg
                    #print("warning for =", i, "th IHA distance and speed = ", distance, magnOfSpeed)
        #distance, magnOfSpeed = brake_var
        print("brake_temp", brake_temp)
        print("brake_var", brake_var)
        if(brake_var > 90):
            brake_var = 90
        if len(self.uav_msg['uav_link']) == 1:
            brake_var = 90
        self.maxSpeed = brake_var
        if self.preheading != 380:
            headingdiff = self.preheading - self.pose[3]
            self.headingacc = headingdiff / self.timediff
            print("heading ivme =", self.headingacc)
        self.preheading = self.pose[3] # self.oldheading ile cakismasin diye farkli isim koyuldu
        #self.maxSpeed = abs(brake_temp / 5.6)
        #self.maxSpeed = 90 - self.maxSpeed
        print("max speed =", self.maxSpeed)
        print("real speed =", self.uav_msg['active_uav']['x_speed'], self.uav_msg['active_uav']['y_speed'])
        #self.brakeMagnitude = magnOfSpeed - self.maxSpeed
        #self.brakeAngle = math.degrees(math.atan2(distancex, -distancey))
        #print("total brake(angle, magnitude) =", self.brakeAngle, self.brakeMagnitude)

    def force_vector_calc(self):
        ux = 0
        uy = 0
        for i in range(len(self.uav_msg['uav_link'])):
            uav_name = self.uav_msg["uav_link"][i].keys()
            uav_name = uav_name[0][4:]
            if int(uav_name) != self.uav_id and self.uav_msg['uav_formation']['u_b'] - 10 >= abs(self.uav_msg["uav_link"][i].values()[0]["altitude"] - self.pose[2]):
                distance = math.sqrt((self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])**2 + (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])**2)
                distancex, distancey = (self.uav_msg["uav_link"][i].values()[0]["location"][0] - self.pose[0])/distance, (self.uav_msg["uav_link"][i].values()[0]["location"][1] - self.pose[1])/distance
                u = self.ljp(distance, self.LJP_EPSILON, self.LJP_SIGMA)
                print("col avo id, distance and force=", int(uav_name), distance, u)
                ux = ux + u*distancex
                uy = uy + u*distancey
        self.collisionAngle = math.atan2(ux, -uy)
        self.collisionAngle = math.degrees(self.collisionAngle)
        self.collisionMagnitude = math.hypot(ux,uy)
        #print("avoidance force = " , ux , uy)
        #print("total vector(angle, magnitude) = ", self.collisionAngle, self.collisionMagnitude)

    def time_calc(self):
        if(self.temp >= 1):
            self.timediff = self.uav_msg['sim_time'] - self.time
            print("timediff for loc", self.timediff)
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
            #print("average x and y = ", self.averagex, self.averagey)
            #print("timediff = ", self.timediff)
            #print("tahmini yer degisirme = ", self.instantxdiff, self.instantydiff)
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
                print("formation target =", self.target_position[0], self.target_position[1])
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
        #print(self.formation[self.formation_type])

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
        '''
        if target_position[2] < 1.0:
            target_position[2] = 1.0
        if dist > 30.0:
        	x_speed = 20.0
        '''
        self.target_speed = [x_speed *  1.00133, y_speed *  1.00133]
        self.send_move_cmd(x_speed, y_speed, target_angle, target_position[2])

    def reached(self, dist):
        if dist < 3:
            return True
        else:
            return False

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

    '''
    def noise_filter(self,data):
        loc_y=data["active_uav"]["location"][1]
        loc_x=data["active_uav"]["location"][0]
        if len(self.defterx)<=12:
            self.defterx.append(loc_x)
            self.deftery.append(loc_y)
        if len(self.defterx)==12:
            self.defterx.pop(0)
            self.deftery.pop(0)
        if data["uav_guide"]["gps_noise_flag"] == True:

            inds = arange(0,11)
            slope, intercept, r_value, p_value, std_err = stats.linregress(inds,self.defterx)
            linex = slope*inds+intercept
            tahminX=linex[5]
            slope, intercept, r_value, p_value, std_err = stats.linregress(inds,self.deftery)
            liney = slope*inds+intercept
            tahminY=liney[5]
            print("tahmin",tahminX,tahminY)
            print("reel",loc_x,loc_y)
            return [tahminX,tahminY]
    '''
    def updateUAVLink(self):
        data = self.prepareUAVLink()
        if len(self.defter) == 0:
            #print(len(data))
            for i in range(len(data)):
                list = []
                self.defter.append(list)
        #print("bum", self.defter)
        if len(self.defter[0]) == 0:
            for i in range(self.params['uav_count']):
                #veriyi kullan
                list = ['x']
                self.defter[i].append(list)
                list = ['y']
                self.defter[i].append(list)
        print("bum", self.defter)
        #loc_x=data[i][0]
        #loc_y=data[i][1]
        #self.defter[iha_id][x or y][paket sirasi]
        #print(self.defter[i])
        for i in range(self.params['uav_count']):
            if len(self.defter[i][0])<=12:
                #print("defter", self.defter)
                #print("i",i)
                self.defter[i][0].append(data[i][0])
                self.defter[i][1].append(data[i][1])
            if len(self.defter[i][0]) == 12:
                #print("defter", self.defter)
                #print("i",i)
                self.defter[i][0].pop(0)
                self.defter[i][1].pop(0)
                self.temp = self.temp + 1
            #print(self.defter)
            if self.uav_msg['uav_guide']['gps_noise_flag'] == True:
                inds = arange(0,11) # bu nedir bilmorm
                #print(self.defter[i][0])
                slope, intercept, r_value, p_value, std_err = stats.linregress(inds, self.defter[i][0])
                linex = slope*inds+intercept
                tahminX=linex[5]
                slope, intercept, r_value, p_value, std_err = stats.linregress(inds, self.defter[i][1])
                liney = slope*inds+intercept
                tahminY=liney[5]
                print(i, "numarali ihanin tahmini konumu = ", tahminX, tahminY)
                self.defter[i][0][9] = tahminX
                self.defter[i][1][9] = tahminY
                #print("reel", loc_x, loc_y)

    def prepareUAVLink(self):
        data = []
        if len(data) == 0:
            #print("bum")
            for i in range(self.params['uav_count']):
                loc = []
                loc.append(0)
                loc.append(0)
                loc.append(-1)
                data.append(loc);
        for i in range(len(self.uav_msg['uav_link'])):
            uav_name = self.uav_msg["uav_link"][i].keys()
            uav_name = uav_name[0][4:]
            if int(uav_name) != self.uav_id:
                data[int(uav_name)] = [self.uav_msg["uav_link"][i].values()[0]["location"][0], self.uav_msg["uav_link"][i].values()[0]["location"][1], self.uav_msg["uav_link"][i].values()[0]["altitude"]]
        #print(data)
        return data

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
    def point_control(self,zones,point):
        i=0

        tik=0
        for i in range (len(zones)):
            bolge=mpltPath.Path(zones[i])
            inside=bolge.contains_points([point])
            if inside[0]==True:
                tik=1
                pack=[inside,zones[i]]
                return pack

            if tik==0:
                pack=[inside,0]

        return pack

    def slice_control(self,dilim,bas,sinir,ustsinir,zones,data,px):
        pack=[]
        start=1

        for i in range(int(bas),sinir,-px):
            make_point=[dilim,i]
            inside=self.point_control(zones,make_point)

            if (start==1) and (inside[0][0]==False):
                upper=make_point
                start=0

            if start==0:

                if (inside[0][0]==True) or (i-px<=sinir):
                    if inside[0][0]==True:
                        ustsinir=inside[1]
                    lower=make_point
                    if inside[0][0]==True:
                        pack=[upper,lower,inside[1],ustsinir]

                        return pack
                    if inside[0][0]==False:
                        pack=[upper,lower,0,ustsinir]

                        return pack
        return pack

    def unpack(self, zone, dilim, data, px):
        top=data["world_boundaries"]
        top=max(top)
        zone_stop=-9999999999
        for i in range(len(zone)):
            if zone[i][0]>zone_stop:
                zone_stop=zone[i][0]
        top=top[1]
        while True:
            make_point=[dilim,top]
            top=top-px
            pack=self.point_control([zone],make_point)

            if pack[0][0]==True:
                start=[make_point,zone,zone_stop]
                break
        return start

    def BCD(self,zones,area,data,px):
        start=area[3]
        dilim=start[0]
        bas=start[1]
        sinir=min(area)
        sinir=sinir[1]
        altsinir=[]
        upper=[]
        lower=[]
        solsinir=max(area)
        solsinir=solsinir[0]
        ustsinir=max(area)
        ustsinir=ustsinir[0]
        cells=[]
        cell=[]
        stack_point=[]
        stack_area=[]
        stack_stop=[]
        start=1
        denied_start=0
        while True:
            paket=self.slice_control(dilim,bas,sinir,ustsinir,zones,data,px)

            if start==1:

                altsinir=paket[2]
                ustsinir=paket[3]
                upper.append(paket[0])
                lower.append(paket[1])
                start=0
                dilim=dilim+px
            if start==0 and altsinir==paket[2] and ustsinir==paket[3]:

                upper.append(paket[0])
                lower.append(paket[1])
                dilim=dilim+px


            if start==0:

                if (altsinir!=paket[2]) or (ustsinir!=paket[3]):
                    if (altsinir!=paket[2]) and (ustsinir!=0):
                        denied_start=dilim
                    temp=[]
                    temp=lower[::-1]


                    cell=temp+upper
                    cells.append(cell)
                    cell=[]
                    lower=[]
                    upper=[]
                    altsinir=paket[2]
                    ustsinir=paket[3]


                if paket[2]!=0:

                    new_start=self.unpack(paket[2],denied_start,data,px)
                    if new_start[0] not in stack_point:
                        stack_area.append(new_start[1])
                        stack_point.append(new_start[0])
                        stack_stop.append(new_start[2])


                if dilim>solsinir and len(stack_point)!=0:
                    temp=[]
                    temp=lower[::-1]


                    cell=temp+upper
                    cells.append(cell)
                    cell=[]
                    lower=[]
                    upper=[]
                    altsinir=paket[2]
                    ustsinir=paket[3]

                    pop=stack_point.pop()
                    stack_area.pop()
                    zone_stop=stack_stop.pop()
                    dilim=pop[0]
                    bas=pop[1]
                    solsinir=int(zone_stop)


                if dilim>solsinir and len(stack_point)==0:
                    temp=[]
                    temp=lower[::-1]

                    cell=temp+upper
                    cells.append(cell)

                    break
        return cells

    def inDeniedZone(self, p, deniedZones):
        for polygon in deniedZones:
            path = mpltPath.Path(polygon)
            if path.contains_points(p):
                return True
        return False

    def notInDeniedZone(self, p, deniedZones):
        for polygon in deniedZones:
            path = mpltPath.Path(polygon)
            if path.contains_points(p):
                return False
        return True

    def forAll(self,l):
        for i in l:
            print(i)

    def dist(self,position1, position2):
        sum = 0
        for i in range(len(position1)):
            diff = position1[i]-position2[i]
            sum += diff * diff
        return math.sqrt(sum)



    def makeClusters(self, data):
        for building in data['special_assets']:
            if building['type'] == 'tall_building':
                for p in building['locations']:
                    if self.notInDeniedZone([p],data["denied_zones"]):
                        self.special_assets.append({
                            'p':[
                                float(p[0] + self.position_offset),
                                float(p[1] + self.position_offset)
                            ],
                            'c': 0
                        })
            else:
                self.special_assets.append({
                    'p':[
                        float(building['location']['x'] + self.position_offset),
                        float(building['location']['y'] + self.position_offset)
                    ],
                    'c': 0
                })
                self.special_assets.append({
                    'p':[
                        float(building['location']['x'] + self.position_offset),
                        float(building['location']['y'] + self.position_offset)
                    ],
                    'c': 0
                })
        self.cluster_count
        self.cluster_element_treshold
        for i in range(len(self.special_assets)):
            neighbour_index_list = [i]
            base_point = self.special_assets[i]
            if not self.cluster_count:
                for j in range(len(self.special_assets)):
                    if(j != i):
                        d = self.dist(base_point['p'], self.special_assets[j]['p'])
                        if d <= self.bridge_length:
                            neighbour_index_list.append(j)
                if len(neighbour_index_list) > self.cluster_element_treshold:
                    self.cluster_count = self.cluster_count + 1
                    for j in neighbour_index_list:
                        self.special_assets[j]['c'] = self.cluster_count
            else:
                if base_point['c']:
                    for j in range(len(self.special_assets)):
                        if(j != i) and (not self.special_assets[j]['c']):
                            d = self.dist(base_point['p'], self.special_assets[j]['p'])
                            if d <= self.bridge_length:
                                self.special_assets[j]['c'] = base_point['c']
                else:
                    for j in range(len(self.special_assets)):
                        d = self.dist(base_point['p'], self.special_assets[j]['p'])
                        if d <= self.bridge_length:
                            if(j != i) and (self.special_assets[j]['c']):
                                self.special_assets[i]['c'] = self.special_assets[j]['c']
                                break
                            neighbour_index_list.append(j)
                    if self.special_assets[i]['c']:
                        continue
                    elif len(neighbour_index_list) > self.cluster_element_treshold:
                        self.cluster_count += 1
                        for j in neighbour_index_list:
                            self.special_assets[j]['c'] = self.cluster_count

    def normalPos(self,p):
        return [p[0] - self.position_offset, p[1] - self.position_offset]

    def unpacked_cluster(self,clusters,width):
        mask_for_cluster=[]
        for i in range(len(clusters)):
            mask_for_cluster.append([])
        for i in range(len(clusters)):
            for j in range(len(clusters[i])):
                clusters[i][j]
                tmp=[clusters[i][j][0]-float((width/2)),clusters[i][j][1]+float((width/2))]
                tmps=[[tmp[0],tmp[1]],[tmp[0]+width,tmp[1]],[tmp[0]+width,tmp[1]-width],[tmp[0],tmp[1]-width]]
                mask_for_cluster[i].append(tmps)
        return mask_for_cluster

    def findPath(self,hashno,allpoints_dict,px):
        hashno=str(hashno)
        points=allpoints_dict[hashno]
        start=points.pop(0)
        make_point=start
        new_path=[]
        new_path.append(start)
        back=0
        while len(points)!=0:
            if [make_point[0],make_point[1]+px] in points:
                back=0
                points.remove([make_point[0],make_point[1]+px])
                new_path.append([make_point[0],make_point[1]+px])
                make_point=[make_point[0],make_point[1]+px]
                continue

            elif [make_point[0]+px,make_point[1]] in points:
                back=0
                points.remove([make_point[0]+px,make_point[1]])
                new_path.append([make_point[0]+px,make_point[1]])
                make_point=[make_point[0]+px,make_point[1]]
                continue

            elif [make_point[0],make_point[1]-px] in points:
                back=0
                points.remove([make_point[0],make_point[1]-px])
                new_path.append([make_point[0],make_point[1]-px])
                make_point=[make_point[0],make_point[1]-px]
                continue
            elif [make_point[0]-px,make_point[1]] in points:
                back=0
                points.remove([make_point[0]-px,make_point[1]])
                new_path.append([make_point[0]-px,make_point[1]])
                make_point=[make_point[0]-px,make_point[1]]
                continue

            else:
                try:

                    back=back+1
                    make_point=new_path[-(2*back)]
                    new_path.append([make_point[0],make_point[1]])
                except:
                    break

        return new_path
    def sortSubareas(self,subareas,maxQ):
        sub_dist={}
        point=maxQ[0]
        point=np.array(point)
        x = point[:,0]
        y = point[:,1]
        center = [sum(x) / len(point), sum(y) / len(point)]
        for i in range(len(maxQ)-1):
            point=maxQ[i+1]
            point=np.array(point)
            x = point[:,0]
            y = point[:,1]
            temp_center = [sum(x) / len(point), sum(y) / len(point)]
            center=[(center[0]+temp_center[0])/2,(center[1]+temp_center[1])/2]

        i=0
        for i in range(len(subareas)):
            hashno=str(hash(str(subareas[i])))
            point=subareas[i]
            point=np.array(point)
            x = point[:,0]
            y = point[:,1]
            sub_center = [sum(x) / len(point), sum(y) / len(point)]
            distofcenter=self.dist(sub_center,center)
            sub_dist[hashno]=distofcenter
        i=0
        temp=subareas
        hashkeys=[]
        for i in range(len(temp)):
            hashno=str(hash(str(temp[i])))
            hashkeys.append(hashno)


        hashlist=[]
        temp=0
        for j in range(len(hashkeys)):
            for i in range(len(hashkeys)):

                low=-sub_dist[hashkeys[i]]
                if temp>low:
                    temp=low
                    hashkey=hashkeys[i]
                    index=i

            if hashkey not in hashlist:
                hashlist.append(hashkey)

                sub_dist.pop(hashkey)
                hashkeys.pop(index)

                temp=0
        return hashlist
    def angle_between(self,p1, p2):
        ang1 = np.arctan2(*p1[::-1])
        ang2 = np.arctan2(*p2[::-1])
        return np.rad2deg((ang1 - ang2) % (2 * np.pi))

    def findTurnSide(self,denied,instantloc,finish):
        if denied==0:
            return 0
        point=denied
        point=np.array(point)
        xx = point[:,0]
        yy = point[:,1]
        centerofdenied = [sum(xx) / len(point), sum(yy) / len(point)]

        x=instantloc[0]
        y=instantloc[1]
        new_centerofdenied=[centerofdenied[0]-x,centerofdenied[1]-y]
        new_finish=[finish[0]-x,finish[1]-y]
        angle=self.angle_between(new_finish,new_centerofdenied)
        return angle


    def findRotationPath(self,deniedzones,allpath,start,finish,px):
        deltax=finish[0]-start[0]
        deltay=finish[1]-start[1]
        distance=dist(start,finish)
        rotationPath=[]
        distance_px=distance/10
        x_px=deltax/distance_px
        y_px=deltay/distance_px
        start_t=start
        A=0
        B=0
        C=0
        #planA
        i=0
        rotationPath.append(start_t)
        for i in range(int(distance_px)):
            make_point=[start_t[0]+x_px,start_t[1]+y_px]
            start_t=make_point
            rotationPath.append(make_point)
            distance=self.dist(start_t,finish)
            pack=self.point_control(deniedzones,make_point)
            if pack[0]==True:
                A=1
        tempA=rotationPath
        rotationPath=[]
        start_t=start
        #planB
        i=0
        rotationPath.append(start_t)
        for i in range(int(distance_px)):
            make_point=[start_t[0]+x_px,start_t[1]]
            start_t=make_point
            rotationPath.append(make_point)
            distance=self.dist(start_t,finish)
            pack=self.point_control(deniedzones,make_point)
            if pack[0]==True:
                B=1
            #print(distance)
        i=0
        for i in range(int(distance_px)):
            make_point=[start_t[0],start_t[1]+y_px]
            start_t=make_point
            rotationPath.append(make_point)
            distance=self.dist(start_t,finish)
            pack=self.point_control(deniedzones,make_point)
            if pack[0]==True:
                B=1
            #print(distance)
        tempB=rotationPath
        rotationPath=[]
        #planC
        start_t=start
        i=0
        rotationPath.append(start_t)
        for i in range(int(distance_px)):
            make_point=[start_t[0],start_t[1]+y_px]
            start_t=make_point
            rotationPath.append(make_point)
            distance=self.dist(start_t,finish)
            pack=self.point_control(deniedzones,make_point)
            if pack[0]==True:
                C=1
            #print(distance)
        i=0
        for i in range(int(distance_px)):
            make_point=[start_t[0]+x_px,start_t[1]]
            start_t=make_point
            rotationPath.append(make_point)
            distance=dist(start_t,finish)
            pack=self.point_control(deniedzones,make_point)
            if pack[0]==True:
                C=1
        tempC=rotationPath
        rotationPath=[tempA,tempB,tempC]
        if A==0:
            rotationPath=tempA
        elif B==0:
            rotationPath=tempB
        elif C==0:
            rotationPath=tempC
        start_t=start
        if A==1 and B==1 and C==1:
            #planA
            rotationPath=[]
            i=0
            dodge=0
            rotationPath.append(start_t)
            whiledist=self.dist(start,finish)
            while whiledist>10:
                #come=dist(finish,start_t)
                aci=math.atan2(start_t[0]-finish[0],start_t[1]-finish[1])
                aci=math.degrees(aci)
                if dodge==0:
                    make_point=[start_t[0]+x_px,start_t[1]+y_px]
                    pack=self.point_control(deniedzones,make_point)
                if dodge==0 and pack[0]==False:
                    rotationPath.append(make_point)
                    start_t=make_point
                if pack[0]==True:
                    dodge=1
                if dodge==1:
                    make_point=[start_t[0],start_t[1]]
                    angle=self.findTurnSide(pack[1],start_t,finish)
                    if aci<=-45 and aci>=-135:
                        if angle>0 and angle<180:
                            point_pack=[[start_t[0]+10,start_t[1]],[start_t[0],start_t[1]+10]]
                        if angle<360 and angle>180:
                            point_pack=[[start_t[0]+10,start_t[1]],[start_t[0],start_t[1]-10]]
                    if aci<=45 and aci>=-45:
                        if angle>0 and angle<180:
                            point_pack=[[start_t[0]+10,start_t[1]],[start_t[0],start_t[1]-10]]
                        if angle<360 and angle>180:
                            point_pack=[[start_t[0]-10,start_t[1]],[start_t[0],start_t[1]-10]]
                    if aci>=45 and aci<=135:
                        if angle>0 and angle<180:
                            point_pack=[[start_t[0]-10,start_t[1]],[start_t[0],start_t[1]-10]]
                        if angle<360 and angle>180:
                            point_pack=[[start_t[0],start_t[1]+10],[start_t[0]-10,start_t[1]]]
                    if (aci>=135 and aci<=180) or (aci<=-135 and aci>=-180):
                        if angle>0 and angle<180:
                            point_pack=[[start_t[0],start_t[1]+10],[start_t[0]-10,start_t[1]]]
                        if angle<360 and angle>180:
                            point_pack=[[start_t[0]+10,start_t[1]],[start_t[0],start_t[1]+10]]
                    angle=self.findTurnSide(pack[1],start_t,finish)
                    a=0
                    for i in range(len(point_pack)):
                        pack1=self.point_control(deniedzones,point_pack[i-a])
                        if pack1[0]==True:
                            point_pack.pop(i-a)
                            a=a+1
                    distt=self.dist(finish,point_pack[0])
                    lowest=distt
                    loc=point_pack[0]
                    for i in range(len(point_pack)):
                        pack2=self.point_control(deniedzones,point_pack[i])
                        if lowest >self.dist(finish,point_pack[i]) and pack2[0][0]==False:
                            lowest=self.dist(finish,point_pack[i])
                            loc=point_pack[i]
                    start_t=loc
                    whiledist=self.dist(start_t,finish)
                    rotationPath.append(start_t)
        return rotationPath

    def path_planning(self, data):

        tall_index = 0
        while(data['special_assets'][tall_index]['type'] != 'tall_building'):
            tall_index += 1
        self.special_assets = []
        tall_count = len(data['special_assets'][tall_index]['locations'])

        #if data['special_assets'][tall_index]['width'][0] > data['special_assets'][tall_index]['width'][1]:
        #    bridge_length = data['special_assets'][tall_index]['width'][0] * 100
        #else:
        #    bridge_length = data['special_assets'][tall_index]['width'][1] * 2.5
        self.bridge_length = 100.0
        self.cluster_count = 0
        self.cluster_element_treshold = 3

        deniedZones = data['denied_zones']
        self.position_offset = float(data['world_length'] / 2)
        color_cursor = 0
        colors = [
            'black',
            'red',
            'green',
            'blue',
            'magenta',
            'yellow',
            'cyan',
        ]
        #bolgeler kumelendi "special_assets" diye
        self.makeClusters(data)
        for i in range(len(self.special_assets)):
            self.special_assets[i]['p'] = self.normalPos(self.special_assets[i]['p'])


        clusters=[]
        for i in range(self.cluster_count+1):
            clusters.append([])
        for i in self.special_assets:
            clusters[i["c"]].append(i["p"])

        mask_for_cluster=self.unpacked_cluster(clusters,75)
        merge_tall=[]
        temp_mask_for_cluster=[]
        for j in range(len(mask_for_cluster)):
            for i in range(len(mask_for_cluster[j])):
                merge_tall=merge_tall+mask_for_cluster[j][i]
            merge_tall=np.array(merge_tall)
            temp_mask_for_cluster.append(merge_tall)
            merge_tall=[]
        #kumelenme bitti

        #kritik bolgeler "maxQ_areas" adi altinda kabuklandi
        i=0
        maxQ_Areas=[]
        for i in range(1,len(temp_mask_for_cluster)):
            points = temp_mask_for_cluster[i]
            hull = ConvexHull(points)
            temp=list(points[hull.vertices])
            maxQ_Areas.append(temp)

        #uzun bina lokasyonlari polygon icin ayarlandi
        tall_locs_=[]
        for t in range(len(data["special_assets"])):
            if data["special_assets"][t]["type"]=="tall_building":
                #buyuklugu 10 arttirildi binalarin
                tall_width=max(data["special_assets"][t]["width"])+10
                tall_locs=data["special_assets"][t]["locations"]
                for i in range(len(tall_locs)):
                    tmp=[tall_locs[i][0]-(tall_width/2),tall_locs[i][1]+(tall_width/2)]
                    tmps=[[tmp[0],tmp[1]],[tmp[0]+tall_width,tmp[1]],[tmp[0]+tall_width,tmp[1]-tall_width],[tmp[0],tmp[1]-tall_width]]
                    tall_locs_.append(tmps)
        #hastane lokasyonlari polygon icin ayarlandi
        h_locs=[]
        i=0
        h_width=60
        h_height=80
        for i in range(len(data["special_assets"])):
            if data["special_assets"][i]["type"]=="hospital":
                xtemp=[data["special_assets"][i]["location"]["x"],data["special_assets"][i]["location"]["y"]]
                h_tmp=[data["special_assets"][i]["location"]["x"]-(h_width/2),data["special_assets"][i]["location"]["y"]+(h_height/2)]
                h_tmps=[[h_tmp[0],h_tmp[1]],[h_tmp[0]+h_width,h_tmp[1]],[h_tmp[0]+h_width,h_tmp[1]-h_height],[h_tmp[0],h_tmp[1]-h_height]]
                h_locs.append(h_tmps)

        # path icin girilmemesi gereken bolgeler olusturuldu denied zone ,uzun binalar , ve hastaneler.
        data["denied_zones"]
        all_denied=[]
        all_denied=tall_locs_+h_locs+data["denied_zones"]

        # hucreleme islemi icin kumenin disinda kalan yapilar ve denied zone farkli bir liste yapildi
        denied_for_bcd=[]
        denied_for_bcd=data["denied_zones"]+mask_for_cluster[0]+maxQ_Areas
        area=data["world_boundaries"]
        subareas=self.BCD(denied_for_bcd,area,data,5)
        #subareas adi altinda hucreler olustu
        i=0
        temp=[]
        subarea_dict={}
        for i in range (len(subareas)):
            if len(subareas[i])>=2:
                temp.append(subareas[i])
        subareas=temp
        for i in range(len(subareas)):
            subarea_dict[str(hash(str(subareas[i])))]=subareas[i]



        # olusan kritik bolgelerin yakinliklarina gore hucreler siralandi
        #buyukten kucuge
        temp=[]
        sorted_subareas=self.sortSubareas(subareas,maxQ_Areas)
        for i in range(len(sorted_subareas)):
            temp.append(subarea_dict[sorted_subareas[i]])
        sorted_subareas=temp
        #kucukten buyuge
        sorted_subareas=sorted_subareas[::-1]


        #hucrelerin basina arastirilmasi oncelikli kumelenen bolgeler liste basina eklendi ki oncelik ordan baslasin
        sorted_subareas=maxQ_Areas+sorted_subareas
        subareas=maxQ_Areas+subareas



        #bu olusan bolgeler icin boktalar yerlestirildi rota icin.
        path_for_subareas={}
        temp=[]
        top_right=max(data["world_boundaries"])
        for path_point in range(0,data["world_length"],10):
            for path_point1 in range (0,data["world_width"],10):
                make_point=[top_right[0]-path_point,top_right[1]-path_point1]
                ekle=1
                for i in range (len(all_denied)):
                    paths=mpltPath.Path(all_denied[i])
                    inside=paths.contains_points([make_point])
                    if inside==True:
                        ekle=0
                    elif inside==False:
                        continue
                if ekle==1:
                    pack=self.point_control(subareas,make_point)
                    hashh=hash(str(pack[1]))
                    if str(str(hashh)) in path_for_subareas:

                        temp=path_for_subareas[str(hashh)]
                        temp.append(make_point)
                        path_for_subareas[str(hashh)]=temp
                    else:
                        path_for_subareas[str(hashh)]=[make_point]


        #tum bolgelere yol cizildi
        for i in range(len(subareas)):
            hashh=hash(str(subareas[i]))
            new_path=self.findPath(hashh,path_for_subareas,10)
            path_for_subareas[str(hashh)]=new_path

        self.sorted_subareas=sorted_subareas
        self.path_for_subareas=path_for_subareas
