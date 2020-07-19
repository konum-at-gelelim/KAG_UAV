from base.base_uav import BaseUAV
import math
import random
import util


class SampleUAV(BaseUAV):

    def initialize(self):
        self.target_position = [2000,-1000,90]
        self.fallback=False
        self.start_loc=None

    def act(self):
        if self.start_loc==None:
            self.start_loc=[self.uav_msg["active_uav"]["location"][0],self.uav_msg["active_uav"]["location"][1]]
        self.amifallback()
        if self.fallback:
            print("geridonn")
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        print(self.uav_msg["active_uav"]["x_speed"],self.uav_msg["active_uav"]["y_speed"])
        # if self.uav_msg['uav_guide']['dispatch']:
        #     print('uav_guide.dispath is True')
        if self.reach_to_target():
            self.send_move_cmd(0, 0, target_angle, target_position[2])


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
        x_speed,y_speed=self.getXY(target_position[0],target_position[1],x_speed)
        self.send_move_cmd(x_speed, y_speed, target_angle, target_position[2])

    def process_uav_msg(self):
        self.pose = [self.uav_msg['active_uav']['location'][0],
                     self.uav_msg['active_uav']['location'][1],
                     self.uav_msg['active_uav']['altitude'],
                     self.uav_msg['active_uav']['heading']]
        print('x:' + str(self.pose[0]) + ' y:' + str(self.pose[1]) +
              ' altitude:' + str(self.uav_msg['active_uav']['altitude']) +
              ' fuel:' + str(self.uav_msg['active_uav']['fuel_reserve']))


##################################saha ici hesaplanmadi #############################################
    def amifallback(self):
        fuel=self.uav_msg['active_uav']["fuel_reserve"]
        if self.start_loc==None:
            pass
        if self.fallback==False:
            knot=20/3
            dist= util.dist(self.start_loc,[self.uav_msg['active_uav']['location'][0],self.uav_msg['active_uav']['location'][1]])
            dist=dist/1.852 #knot to kmh
            fuel_=dist*knot #aradaki knot mesafe * knot basina harcanan yakit.

            if fuel>fuel_:
                pass
            if fuel<fuel_:
                self.fallback=True
        if self.fallback==True:
            pass

    def getXY(self,x,y,speed):
        head=self.uav_msg["active_uav"]["heading"]
        targetAngle=self.findAngle(x,y)
        if targetAngle <0:
            targetAngle=360+targetAngle
        head=targetAngle-head
        print(head)
        #target_position=[x,y]
        #dist = util.dist(target_position, self.pose)
        head=math.radians(head)
        yy=math.sin(head)
        xx=math.cos(head)
        top=math.sqrt(xx**2+yy**2)
        value=speed/top
        xx=xx*value
        yy=yy*value
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
