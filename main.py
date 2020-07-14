import sys
from kek_uav import KekUAV


default_ip = '127.0.0.1'
default_port = 5672


def get_uav_id():
    uav_id = sys.argv[1]
    try:
        uav_id = int(uav_id)
    except ValueError:
        print('ERROR: Invalid UAV id:' + sys.argv[1])
        sys.exit(1)
    return str(uav_id)


def get_ip():
    if len(sys.argv) > 2:
        return sys.argv[2]
    else:
        return default_ip


def get_port():
    if len(sys.argv) > 3:
        port_num = sys.argv[3]
        try:
            port_num = int(port_num)
        except ValueError:
            print('ERROR: invalid port number:' + sys.argv[3])
            sys.exit(2)

        return port_num
    else:
        return default_port


if __name__ == '__main__':
    # uav id al
    uav_id = get_uav_id()
    ip = get_ip()
    port = get_port()
    uav = KekUAV(uav_id, ip, port)
    uav.start_listening()