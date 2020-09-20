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
    
def uav_injured_center_dist_sort_method(e):
    return e['injured_center_dist']
    
def uav_id_sort_method(e):
    return e['id']

deniedZones = params['denied_zones']
def notInDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return False
    return True


world_boundaries = params['world_boundaries']
def idle_parkour_gen():
    xmin = world_boundaries[0][0]
    ymin = world_boundaries[0][1]
    xmax = world_boundaries[0][0]
    ymax = world_boundaries[0][1]
    result_list = []
    for i in range(1, len(world_boundaries)):
        if world_boundaries[i][0] < xmin:
            xmin = world_boundaries[i][0]
        elif world_boundaries[i][0] > xmax:
            xmax = world_boundaries[i][0]
        if world_boundaries[i][1] < ymin:
            ymin = world_boundaries[i][1]
        elif world_boundaries[i][1] > ymax:
            ymax = world_boundaries[i][1]
    world_center = [(xmax + xmin) / 2.0, (ymax + ymin) / 2.0]
    for i in range(len(world_boundaries)):
        temp_point = world_boundaries[i]
        temp_point[0] -= (temp_point[0] - world_center[0]) / 2.0
        temp_point[1] -= (temp_point[1] - world_center[1]) / 2.0
        if notInDeniedZone([temp_point]):
            result_list.append(temp_point)
    if len(result_list) != 0:
        return result_list
    elif notInDeniedZone([world_center]):
        return [world_center, self.pose[:2]]
    else:
        return world_boundaries[0]
    
#dummy
def move_to_target(target, task):
    print('moving to ' + str(target) + ' coordinate to serve' + ' with task of ' + str(task))

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
uav_id = 7
telecom_radius = 250.0
telecom_diameter = telecom_radius * 2.0
world_width = 1000.0
world_length = 1000.0
the_n = 1
DEFAULT = 'D'
YOK = 'YOK'
TELEKOM = 'T'
KURTARMA = 'K'
telecom_center = [0.0, 0.0]
injured_center = [0.0, 0.0]
cluster_count = 0
telecom_cluster_centers = []
telecom_nodes = []
injured_nodes = []
######YAKIT
#işte yakıt methodu

######TELEKOM
for i in range(len(casualties)):
    if casualties[i]['status'] == 'healthy':
        telecom_center[0] += casualties[i]['pose'][0]
        telecom_center[1] += casualties[i]['pose'][1]
        telecom_nodes.append({'pose': tuple(casualties[i]['pose']), 'distance': None, 'cluster': -1})
    else:
        injured_center[0] += casualties[i]['pose'][0]
        injured_center[1] += casualties[i]['pose'][1]
        injured_nodes.append({'pose': tuple(casualties[i]['pose']), 'assigned_uav_id': None})    

telecom_center = [telecom_center[0] / len(telecom_nodes), telecom_center[1] / len(telecom_nodes)]
injured_center = [injured_center[0] / len(injured_nodes), injured_center[1] / len(injured_nodes)]
for i in range(len(telecom_nodes)):
    telecom_nodes[i]['distance'] = dist(telecom_center, telecom_nodes[i]['pose'])
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
            'assign': YOK,
            'injured_center_dist': None
        })



for i in range(len(telecom_cluster_centers)):
    nearest = {'index': None, 'dist': None}  
    for j in range(len(uav_list)):
        if uav_list[j]['assign'] != YOK:
            continue
        d = dist(telecom_cluster_centers[i], uav_list[j]['props']['location'])
        if (d < nearest['dist'] or nearest['dist'] == None) and (uav_list[j]['props']['task'] == TELEKOM):
            nearest['dist'] = d
            nearest['index'] = j
    if nearest['index'] != None:
        uav_list[nearest['index']]['assign'] = TELEKOM + '_' + str(i)




if uav_list[uav_id]['assign'][0] == TELEKOM:
    print(int(uav_list[uav_id]['assign'][2:]))
    move_to_target(telecom_cluster_centers[int(uav_list[uav_id]['assign'][2:])], uav_list[uav_id]['props']['task'])
    print('BITTI BURADA METHOD, RETURN YAZIN BU PRINT YERINE!')
####KURTARMA
for i in range(len(uav_list)):
    if uav_list[i]['assign'] == YOK:
        uav_list[i]['injured_center_dist'] = dist(injured_center, uav_list[i]['props']['location'])
uav_list.sort(reverse=True, key=uav_injured_center_dist_sort_method)

for i in range(len(uav_list)):
    if uav_list[i]['injured_center_dist'] == None:
        break
    nearest = {'index': None, 'dist': None}
    for j in range(len(injured_nodes)):
        if injured_nodes[j]['assigned_uav_id'] != None:
            continue
        d = dist(uav_list[i]['props']['location'], injured_nodes[j]['pose'])
        if (d < nearest['dist'] or nearest['dist'] == None):
            nearest['dist'] = d
            nearest['index'] = j
    if nearest['index'] != None:
        injured_nodes[nearest['index']]['assigned_uav_id'] = uav_list[i]['id']
        uav_list[i]['assign'] = KURTARMA + '_' + str(nearest['index'])
        uav_list[i]['props']['task'] = KURTARMA
uav_list.sort(key=uav_id_sort_method)
if uav_list[uav_id]['assign'][0] == KURTARMA:
    move_to_target(injured_nodes[int(uav_list[uav_id]['assign'][2:])], uav_list[uav_id]['props']['task'])
    print('BITTI BURADA METHOD, RETURN YAZIN BU PRINT YERINE!')
#####IDLE PARKOUR

for i in uav_list:
    print(i['assign'])
print('KEKW')
print(world_boundaries)
idle_parkour = idle_parkour_gen()
print(idle_parkour)






    
