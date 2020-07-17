from base.base_uav import BaseUAV
from math import sin, cos, pi, atan2, hypot
import random
import time
import numpy as np
import math
import random
import util

SIZE = 600
COUNT = 10
SPEED = 100
FOLLOWERS = 4

LJP_EPSILON = 10 #iki atom arasi minimum uzaklik
LJP_SIGMA = 3.3 #kuvvet birimi

class SampleUAV(BaseUAV):

    def initialize(self):
        self.target_position = None
        #self.history = deque(maxlen=64)
    def act(self):
        # bu adimda mevcut mesaj islenecek ve bir hareket komutu gonderilecek.
        self.process_uav_msg()
        # if self.uav_msg['uav_guide']['dispatch']:
        #     print('uav_guide.dispath is True')

        if self.reach_to_target():
            self.target_position = (200,200,90)
            print('random target position x:%f y:%f altitude:%f' %
                  (self.target_position[0], self.target_position[1], self.target_position[2]))
        self.vector_move_to_target(self.target_position)

    def ljp(self,r, epsilon, sigma):
        return 48 * epsilon * np.power(sigma, 12) / np.power(r, 13) \
        - 24 * epsilon * np.power(sigma, 6) / np.power(r, 7)

    def speed(self):
        return 50

    def uav_update(self):
        positionx, positiony = self.pose[0],self.pose[1],
        targetx, targety,targetz = self.target_position
        angle = atan2(targetx-positionx,targety-positiony)
        ux = cos(angle)*self.speed()
        uy = sin(angle)*self.speed()
        print("speed = " , ux , " , " , uy,"\n")
        # ucaklarin konumlarina bakilyor
        for i in range(len(self.uav_msg['uav_link'])):
	    a=self.uav_msg["uav_link"][i].keys()
            uav_name=a[0][4]
            uav_name=str(uav_name)
            if uav_name != self.uav_id:
                tempx ,tempy = self.uav_msg["uav_link"][i].values()[0]["location"][0],self.uav_msg["uav_link"][i].values()[0]["location"][1]
                distance =  hypot(positionx - tempx, positiony - tempy)
                angle = atan2(positiony - tempy,positionx - tempx)
                u = self.ljp(distance, LJP_EPSILON, LJP_SIGMA) # ucaklardan kacabilmek icin kuvvet
                ux += cos(angle)*u
                uy += sin(angle)*u
        angle = atan2(uy,ux)
        magnitude = hypot(ux,uy)
        print("avoidance force = " , ux , " , " , uy,"\n")
        return angle,magnitude

    def set_position(self, position): #onceki noktalar kayidi
        self.position = position
        if not self.history:
            self.history.append(self.position)
            return
        x, y = self.position
        px, py = self.history[-1]
        d = hypot(px - x, py - y)
        if d >= 10:
            self.history.append(self.position)

    def reach_to_target(self):
        if self.target_position is None:
            return True

        thresh = 3.0 # bu degerden daha yakinsak vardik sayiyoruz
        dist = util.dist(self.target_position, self.pose)
        if dist < thresh:
            return True
        else:
            return False


    def vector_move_to_target(self, target_position):
        angle, magnitude = self.uav_update()
        angle = self.uav_msg["active_uav"]["heading"] - angle
        if(angle < 0):
            angle += + 360
        xspeed = cos(angle) * magnitude
        yspeed = sin(angle) * magnitude
        #if dist < 50:
        #    x_speed = dist*0.25
        #    y_speed = dist*0.25
        self.send_move_cmd(xspeed,yspeed,self.uav_msg["active_uav"]["heading"], target_position[2])

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
"""
class UAV(object):
    def __init__(self, position, target):
        self.position = position
        self.target = target
        self.heading = heading
        #self.ljp = 0
        #self.speed
        self.history = deque(maxlen=64)
    #def get_position(self,offset):
        #positionx, positiony, positionz = self.position # z = altitude
        #targetx, targety, targetz = self.target
        #heading = self.heading
        #angle = atan2(targety - positiony, targetx - positionx)
        #altitude_angle = atan2()
        #return (positionx + cos(angle) * offset, positiony + sin(angle) * offset)



    def get_position(self, offset):
        px, py = self.position # position_x and position_y
        tx, ty = self.target # target_x and target_y
        angle = atan2(ty - py, tx - px) # targete olan aci
        return (px + cos(angle) * offset, py + sin(angle) * offset) # cozemedim.


class Model(object):
    def __init__(self, width, height, count):
        self.width = width      # ?
        self.height = height    # ?
        self.bots = self.create_bots(count) # count adet bot uretiliyor

    def create_bots(self, count):
        result = [] # bot listesi
        for i in range(count):
            position = self.select_point() # ?
            target = self.select_point() # ?
            bot = Bot(position, target) # pozisyon ve hedefe gidecek olan bot uretimi.
            result.append(bot) # uretilen bot listeye ekleniyor.
        return result # liste return ediliyor.

    def select_point(self): # topcuklar icin konum uretimi
        cx = self.width / 2.0
        cy = self.height / 2.0
        radius = min(self.width, self.height) * 0.4 # ekran genisligine bagli radius uretimi
        angle = random.random() * 2 * pi # rastgele aci
        x = cx + cos(angle) * radius
        y = cy + sin(angle) * radius
        return (x, y)   #
    def update(self, dt):
        data = [bot.update(self.bots) for bot in self.bots]
        for bot, (angle, magnitude) in zip(self.bots, data):
            speed = min(1, 0.2 + magnitude * 0.8)
            dx = cos(angle) * dt * SPEED * bot.speed * speed
            dy = sin(angle) * dt * SPEED * bot.speed * speed
            px, py = bot.position
            tx, ty = bot.target
            bot.set_position((px + dx, py + dy))
            if hypot(px - tx, py - ty) < 10: # belirtilen noktaya uzaklik 10 olursa.
                bot.target = self.select_point() # yeni rastgele konum uret.
        for bot in self.bots[-FOLLOWERS:]:
            #takipci botlar icin konum uretilmeli ve botlar o noktayi takip etmeli.
            bot.target = self.bots[0].get_position(10)

class Panel(wx.Panel):
    def __init__(self, parent):
        super(Panel, self).__init__(parent)
        self.model = Model(SIZE, SIZE, COUNT)
        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_LEFT_DOWN, self.on_left_down)
        self.Bind(wx.EVT_RIGHT_DOWN, self.on_right_down)
        self.timestamp = time.time()
        self.on_timer()
    def on_timer(self):
        now = time.time()
        dt = now - self.timestamp
        self.timestamp = now
        self.model.update(dt)
        self.Refresh()
        wx.CallLater(10, self.on_timer)
    def on_left_down(self, event):
        self.model.bots[0].target = event.GetPosition()
    def on_right_down(self, event):
        width, height = self.GetClientSize()
        self.model = Model(width, height, COUNT)
    def on_size(self, event):
        width, height = self.GetClientSize()
        self.model = Model(width, height, COUNT)
        event.Skip()
        self.Refresh()
    def on_paint(self, event):
        n = len(COLORS)
        dc = wx.AutoBufferedPaintDC(self)
        dc.SetBackground(wx.BLACK_BRUSH)
        dc.Clear()
        dc.SetPen(wx.BLACK_PEN)
        for index, bot in enumerate(self.model.bots[:n]):
            dc.SetBrush(wx.Brush(COLORS[index]))
            for x, y in bot.history:
                dc.DrawCircle(x, y, 3) # gecmis noktalar ekrana basiliyor
        dc.SetBrush(wx.BLACK_BRUSH)
        for index, bot in enumerate(self.model.bots[:n]):
            dc.SetPen(wx.Pen(COLORS[index]))
            x, y = bot.target
            dc.DrawCircle(x, y, 6) # target ekrana basiliyor
        for index, bot in enumerate(self.model.bots):
            dc.SetPen(wx.BLACK_PEN)
            if index < n:
                dc.SetBrush(wx.Brush(COLORS[index]))
            elif index >= COUNT - FOLLOWERS:
                dc.SetBrush(wx.BLACK_BRUSH)
                dc.SetPen(wx.WHITE_PEN)
            else:
                dc.SetBrush(wx.WHITE_BRUSH)
            x, y = bot.position
            dc.DrawCircle(x, y, 6) # botlar ekrana basliliyor (mainbot,randombots,followerbots)

class Frame(wx.Frame):
    def __init__(self):
        super(Frame, self).__init__(None)
        self.SetTitle('Motion')
        self.SetClientSize((SIZE, SIZE))
        Panel(self)

def main():
    app = wx.App()
    frame = Frame()
    frame.Center()
    frame.Show()
    app.MainLoop()

if __name__ == '__main__':
    main()
"""
