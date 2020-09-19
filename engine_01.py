import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
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
    
def normal_pose(pose):
    return [pose[0] - (world_width / 2.0), pose[1] - (world_length / 2.0)]

def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)

colors = [
    'red',
    'green',
    'blue',
    'magenta',
    'yellow',
    'cyan',
    'black'
]

markers = [
    '.',
    '*',
    '1',
    'x',
]

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
    {'status': 'healthy', 'type': 'casualty', 'pose': (208.884, -74.9161), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (66.041, -67.3497), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (80.0, 54.0), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (80.0, 56.0), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (80.0, 58.0), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (159.851, 68.8703), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (159.774, 67.0955), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (159.982, 64.4953), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (198.297, 77.702), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (201.829, 76.5456), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (218.036, 78.6069), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (223.159, 76.658), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (206.261, 64.038), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (204.259, 59.6497), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (207.0, 62.0), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (203.0, 62.569), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (203.0, 66.0), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (160.302, 61.5684), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (76.3186, 184.819), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (76.0833, 181.986), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (76.1035, 179.44), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (76.1175, 176.663), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (160.087, 62.7893), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (179.066, 79.8941), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (154.65, 198.291), 'in_world': True},
    {'status': 'injured', 'type': 'casualty', 'pose': (155.265, 196.569), 'in_world': True}
]

telecom_radius = 250.0
telecom_diameter = telecom_radius * 2.0
world_width = 1000.0
world_length = 1000.0
the_n = 3
YOK = 'yok'
TELEKOM = 'tel'

#plt.plot(-500, -500, marker='.', color='white')
#plt.plot(-500, 500, marker='.', color='white')
#plt.plot(500, -500, marker='.', color='white')
#plt.plot(500, 500, marker='.', color='white')

center = [0.0, 0.0]
cluster_count = 0
telecom_cluster_centers = []
healthy_count = 0
for i in range(len(casualties)):
    if casualties[i]['status'] == 'healthy':
        healthy_count += 1
        center[0] += casualties[i]['pose'][0]
        center[1] += casualties[i]['pose'][1]

center = [center[0] / float(healthy_count), center[1] / float(healthy_count)]
telecom_nodes = []
for i in range(len(casualties)):
    if casualties[i]['status'] == 'healthy':
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

#for i in telecom_nodes:
#    print i
    
#for i in telecom_cluster_centers:
#    print i
    

for i in telecom_nodes:
    plt.plot(i['pose'][0], i['pose'][1], marker=markers[i['cluster'] // 7], color=colors[i['cluster'] % 7])

plt.show()


uav_position_list = []
for uav_link_count in range(len(uav_msg['uav_link'])):
    for prefix_id in uav_msg['uav_link'][uav_link_count]:
        uav_position_list.append([
            int(prefix_id[4:]),
            float(uav_msg['uav_link'][uav_link_count][prefix_id]['location'][0]),
            float(uav_msg['uav_link'][uav_link_count][prefix_id]['location'][1]),
            float(uav_msg['uav_link'][uav_link_count][prefix_id]['altitude']),
            YOK
        ])

for i in range(len(telecom_cluster_centers)):
    nearest = {'id': -1, 'dist': None}
    for j in range(len(uav_position_list)):
        if uav_position_list[j][4] != YOK:
            continue
        #print(uav_position_list[j][1:4])
        d = dist(telecom_cluster_centers[i], uav_position_list[j][1:])
        if d < nearest['dist'] or nearest['dist'] == None:
            nearest['dist'] = d
            nearest['id'] = j
    uav_position_list[nearest['id']][4] = TELEKOM + '_' + str(i)
    
for i in uav_position_list:
    print i







    
