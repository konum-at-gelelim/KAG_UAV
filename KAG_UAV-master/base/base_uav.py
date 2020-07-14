from base.rabbit_communication import RabbitCommunication


class BaseUAV(object):
    def __init__(self, uav_id, ip, port):
        self.uav_id = uav_id
        # rabbit haberlesme modulu olusturuluyor
        self.comm = RabbitCommunication(host=ip, port=port)
        self.params = self.comm.send_request('scenario_parameters')
        self.initialize()

        self.cmd_queue_name = 'uav_command_queue_' + uav_id

        self.comm.register_to_queue('uav_imu_queue_'+uav_id, self.uav_msg_callback)

    def start_listening(self):
        self.comm.start_listening()

    def uav_msg_callback(self, uav_msg):
        # buraya gelen mesaj ile uav bilgilerini gunceller
        self.uav_msg = uav_msg
        self.act()

    def send_move_cmd(self, x_speed, y_speed, heading, altitude, task='D'):
        cmd = {"x_speed": x_speed, "y_speed": y_speed, "altitude": altitude, "heading": heading, "task": task}
        self.comm.send(self.cmd_queue_name, cmd)

    def initialize(self):
        # bu metod asil takim kodu tarafindan ezilecek
        pass

    def act(self):
        # bu metod asil takim kodu tarafindan ezilecek
        pass
