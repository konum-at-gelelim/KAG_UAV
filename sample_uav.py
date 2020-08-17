from base.base_uav import BaseUAV
import math
import random
import util


class SampleUAV(BaseUAV):

    def initialize(self):
        #[3475,-315,90]
        #[3300,-315,90]
        self.target_position = [2300,-315,90]
        self.fallback=False
        self.start_loc=None
        self.temp = 0
        self.pid_flag=False

    def act(self):

        if self.start_loc==None:
            self.start_loc=[self.uav_msg["active_uav"]["location"][0],self.uav_msg["active_uav"]["location"][1]]
        self.amifallback()
        if self.fallback:
            print("geridonn")
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        print("olusan",self.uav_msg["active_uav"]["x_speed"],self.uav_msg["active_uav"]["y_speed"])
        # if self.uav_msg['uav_guide']['dispatch']:
        #     print('uav_guide.dispath is True')
        #if self.reach_to_target():
        #    self.send_move_cmd(0, 0, self.uav_msg['active_uav']['heading'], self.target_position[2])

        self.move_to_target(self.target_position)
        #print("heading",self.uav_msg["active_uav"]["heading"])
        #print("link heading",self.uav_msg["uav_link"][3]["uav_3"]["heading"])

    def speed_calc(self):
        x = math.cos(math.radians(self.uav_msg["active_uav"]['heading'])) * self.uav_msg["active_uav"]["x_speed"] + math.sin(math.radians(self.uav_msg["active_uav"]['heading'])) * self.uav_msg["active_uav"]["y_speed"]
        y = math.sin(math.radians(self.uav_msg["active_uav"]['heading'])) * self.uav_msg["active_uav"]["x_speed"] + math.cos(math.radians(self.uav_msg["active_uav"]['heading'])) * self.uav_msg["active_uav"]["y_speed"]
        if(self.temp >= 1):
            timediff = self.uav_msg["sim_time"] - self.time

        self.time = self.uav_msg["sim_time"]

        print(self.uav_msg["sim_time"])
        print("speed on x line = " , x)

        if(self.temp >= 1):
            xdiff = self.uav_msg["active_uav"]["location"][0] - self.xloc
            ydiff = self.uav_msg["active_uav"]["location"][1] - self.yloc
            file2=open("test3.txt",'a')
            data=file2.write("degisiklik konum:")
            data=file2.write(str(ydiff))
            data=file2.write("degisiklik zaman:")
            data=file2.write(str(self.uav_msg["sim_time"]))
            data=file2.write("\n")
            file2.close()

            print("diff on x line = ", xdiff)
            hmm=timediff*x
            file=open("test.txt","a")
            data=file.write("zaman*hiz :")
            data=file.write(str(hmm))
            data=file.write("zaman:")
            data=file.write(str(self.uav_msg["sim_time"]))
            data=file.write("\n")
            file.close()

        self.xloc = self.uav_msg["active_uav"]["location"][0]
        self.yloc = self.uav_msg["active_uav"]["location"][1]
        self.temp = self.temp + 1


        self.move_to_target(self.target_position)


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
        x_speed = 90
        #if dist < 50:
            # iha yi yavaslat
        #    x_speed = dist*0.25
        x_speed,y_speed=self.getXY(target_position[0],target_position[1],x_speed)
        self.send_move_cmd(x_speed,y_speed, target_angle, target_position[2])

    def process_uav_msg(self):
        self.pose = [self.uav_msg['active_uav']['location'][0],
                     self.uav_msg['active_uav']['location'][1],
                     self.uav_msg['active_uav']['altitude'],
                     self.uav_msg['active_uav']['heading']]
        #print('x:' + str(self.pose[0]) + ' y:' + str(self.pose[1]) +' altitude:' + str(self.uav_msg['active_uav']['altitude']) +' fuel:' + str(self.uav_msg['active_uav']['fuel_reserve']))


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
