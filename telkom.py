import json
import math

with open('params_sos.json') as f:
    params = json.load(f)

with open('SECRET_SOS.json') as f:
    uav_msg = json.load(f)

def telecom_distance_sort_aux_method(e):
    return e['distance']

def telecom_second_distance_sort_aux_method(e):
    return e['d']
    
def telecom_cluster_sort_method(e):
    return e['cluster']

def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)

casualties = [
    {'status': 'healthy', 'type': 'casualty', 'pose': (170.0, 80.0), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (170.0, 85.0), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (170.0, 90.0452), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (80.0, 50.0), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (80.0, 52.0), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (121.854, -41.8491), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (114.508, -43.4166), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (156.634, -37.4055), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (77.8546, -66.3175), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (80.2377, -61.4033), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (87.1127, -56.6359), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (56.163, -69.9958), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (60.5355, -104.477), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (62.9487, -122.369), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (47.2831, -129.788), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (68.5896, -137.317), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (59.0, -135.0), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (35.2035, -155.298), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (156.138, -46.0915), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (199.87, -28.4846), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (188.678, -32.1271), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (220.411, -30.5651), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (226.107, -32.2785), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (208.198, -66.5461), 'in_world': True},
    {'status': 'healthy', 'type': 'casualty', 'pose': (208.884, -74.9161), 'in_world': True}
]

telecom_radius = 250.0
telecom_diameter = telecom_radius * 2.0
world_width = 1000.0
world_length = 1000.0
the_n = 3
DEFAULT = 'D'
YOK = 'YOK'
TELEKOM = 'T'
KURTARMA = 'K'

center = [0.0, 0.0]
cluster_count = 0
telecom_cluster_centers = []
for i in range(len(casualties)):
    center[0] += casualties[i]['pose'][0]
    center[1] += casualties[i]['pose'][1]

center = [center[0] / 25.0, center[1] / 25.0]
telecom_nodes = []
for i in range(len(casualties)):
    telecom_nodes.append({'pose': tuple(casualties[i]['pose']), 'distance': dist(center, casualties[i]['pose']), 'cluster': -1, 'asigned_id': None})
telecom_nodes.sort(reverse=True, key=telecom_distance_sort_aux_method)

for i in range(len(telecom_nodes)):
    if telecom_nodes[i]['cluster'] != -1:
        continue
    neighbour_list = [{'d': 0.0, 'tni': i}]
    for j in range(len(telecom_nodes)):
        if(i != j) and (telecom_nodes[j]['cluster'] == -1):
            d = dist(telecom_nodes[i]['pose'], telecom_nodes[j]['pose'])
            if d < telecom_diameter:
                neighbour_list.append({'d': d, 'tni': j})

    if len(neighbour_list) >= the_n:
        cluster_count += 1
        neighbour_list.sort(key=telecom_second_distance_sort_aux_method)
        cluster_center_temp = [0.0, 0.0]
        for j in (neighbour_list[:the_n]):
            telecom_nodes[j['tni']]['cluster'] = cluster_count-1
            cluster_center_temp[0] += telecom_nodes[j['tni']]['pose'][0]
            cluster_center_temp[1] += telecom_nodes[j['tni']]['pose'][1]
        cluster_center_temp[0] /= the_n
        cluster_center_temp[1] /= the_n
        telecom_cluster_centers.append(cluster_center_temp)

telecom_nodes.sort(key=telecom_cluster_sort_method)  

uav_list = []
for uav_link_count in range(len(uav_msg['uav_link'])):
    for prefix_id in uav_msg['uav_link'][uav_link_count]:
        uav_list.append({
            'id': int(prefix_id[4:]),
            'props': uav_msg['uav_link'][uav_link_count][prefix_id],
            'assign': YOK
        })


uav_id = 0
for i in range(len(telecom_cluster_centers)):
    nearest = {'id': None, 'dist': None}
    for j in range(len(uav_list)):
        if uav_list[j]['assign'] != YOK:
            continue
        d = dist(telecom_cluster_centers[i], uav_list[j]['props']['location'])
        if (d < nearest['dist'] or nearest['dist'] == None) and (uav_list[j]['props']['task'] == TELEKOM):
            nearest['dist'] = d
            nearest['id'] = j
    if nearest['id'] != None:
        uav_list[nearest['id']]['assign'] = TELEKOM + '_' + str(i)

for i in uav_list:
    print i








    
