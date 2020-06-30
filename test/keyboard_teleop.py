import sys
import termios
import tty
import select
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from base.rabbit_communication import RabbitCommunication


comm = None
queue_name = None


def send_move_cmd(x_speed, y_speed, heading, altitude, task="D"):
    print('x:%f y:%f heading:%f altitude:%f' % (x_speed, y_speed, heading, altitude))
    cmd = {"x_speed": x_speed, "y_speed": y_speed, "altitude": altitude, "heading": heading, "task": task}
    comm.send(queue_name, cmd)


def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def keyboard_loop():
    x_speed = 0
    altitude = 4.5
    heading = 0
    speed_inc_value = 5
    heading_inc_value = 1
    altitude_inc_value = 10
    while True:
        key = get_key()
        if key == '\x1b':
            key = get_key()
            if key == '[':
                key = get_key()
                if key == 'A':
                    # up key
                    x_speed += speed_inc_value
                elif key == 'B':
                    # down key
                    x_speed -= speed_inc_value
                elif key == 'C':
                    # right key
                    heading -= heading_inc_value
                elif key == 'D':
                    # left key
                    heading += heading_inc_value
        elif key == 'w':
            altitude += altitude_inc_value
        elif key == 's':
            altitude -= altitude_inc_value
        elif key == 'q':
            print('quit!')
            break

        send_move_cmd(x_speed, 0, heading, altitude)


if __name__ == '__main__':
    comm = RabbitCommunication()
    queue_name = 'uav_command_queue_' + sys.argv[1]
    comm.register_queue(queue_name)
    settings = termios.tcgetattr(sys.stdin)
    print('press w:increase height, s: decrease height')
    print('up key: forward speed, down key: reduce forward speed')
    print('rigt key: turn right, left key: turn left')
    print('use key Q to QUIT')
    keyboard_loop()

