import sys
import os
import pprint
# a hack to be able to access rabbit communication
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from base.rabbit_communication import RabbitCommunication

comm = None

start = time.time()


def uav_pose_callback(uav_msg):
    global start
    end = time.time()
    print end - start
    start = time.time()
    pprint.pprint(uav_msg)
    # print('pose x:' + str(uav_msg['active_uav']['location'][0]) +
    #       ' y:' + str(uav_msg['active_uav']['location'][1]) +
    #       ' height:' + str(uav_msg['active_uav']['altitude']) +
    #       ' heading:' + str(uav_msg['active_uav']['heading']) +
    #       ' fuel:' + str(uav_msg['active_uav']['fuel_reserve']))


if __name__ == '__main__':
    comm = RabbitCommunication()
    comm.register_to_queue('uav_imu_queue_0', uav_pose_callback)
    params = comm.send_request('scenario_parameters')
    print(params)
    comm.start_listening()
