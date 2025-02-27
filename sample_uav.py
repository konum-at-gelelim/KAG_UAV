from base.base_uav import BaseUAV
import math
import random
import util


class SampleUAV(BaseUAV):

    def initialize(self):
        self.target_position = None

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



