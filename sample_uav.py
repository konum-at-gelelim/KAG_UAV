from base.base_uav import BaseUAV
import math
import random
import util


class SampleUAV(BaseUAV):

    def initialize(self):
        self.target_position = None
        self.fallback=False
        self.start_loc[2]=[self.uav_msg["active_uav"]["location"][0],self.uav_msg["active_uav"]["location"][1]]

    def act(self):
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        # if self.uav_msg['uav_guide']['dispatch']:
        #     print('uav_guide.dispath is True')

        if self.reach_to_target():
            self.target_position = (random.randint(-400, 400),
                               random.randint(-400, 400),
                               random.randint(10, 150))
            print('random target position x:%f y:%f altitude:%f' %
                  (self.target_position[0], self.target_position[1], self.target_position[2]))

        self.move_to_target(self.target_position)

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
        x_speed = 20
        if dist < 50:
            # iha yi yavaslat
            x_speed = dist*0.25
        self.send_move_cmd(x_speed, 0, target_angle, target_position[2])

    def process_uav_msg(self):
        self.pose = [self.uav_msg['active_uav']['location'][0],
                     self.uav_msg['active_uav']['location'][1],
                     self.uav_msg['active_uav']['altitude'],
                     self.uav_msg['active_uav']['heading']]
        print('x:' + str(self.pose[0]) + ' y:' + str(self.pose[1]) +
              ' altitude:' + str(self.uav_msg['active_uav']['altitude']) +
              ' fuel:' + str(self.uav_msg['active_uav']['fuel_reserve']))


##################################saha ici hesaplanmadı #############################################
    def fallback(self,fuel):
        if self.fallback==False:
            knot=20/3
            dist= util.dist(start_loc,[self.uav_msg['active_uav']['location'][0],self.uav_msg['active_uav']['location'][1]])
            dist=dist/1.852 #knot to kmh
            fuel_=dist*knot #aradaki knot mesafe * knot başına harcanan yakıt.

            if fuel>fuel_:
                pass
            if fuel<fuel_:
                self.fallback=True
        if self.fallback==True:
            pass
    def getXY(self,x,y):
        head=self.uav_msg["active_uav"]["heading"]
        targetAngle=self.findAngle(x,y)
        if targetAngle <0:
            targetAngle=360+targetAngle
        head=targetAngle-head
        #target_position=[x,y]
        #dist = util.dist(target_position, self.pose)
        head=math.radians(head)
        xx=math.sin(head)
        yy=math.cos(head)
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
#####################################################################################################
