import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import json
import util
import time

start_time = time.time()
with open('param.json') as f:
    data = json.load(f)


tall_index = 0
while(data['special_assets'][tall_index]['type'] != 'tall_building'):
    tall_index += 1
special_assets = []
tall_count = len(data['special_assets'][tall_index]['locations'])

#if data['special_assets'][tall_index]['width'][0] > data['special_assets'][tall_index]['width'][1]:
#    bridge_length = data['special_assets'][tall_index]['width'][0] * 100
#else:
#    bridge_length = data['special_assets'][tall_index]['width'][1] * 2.5
bridge_length = 100.0
cluster_count = 0
cluster_element_treshold = 3

deniedZones = data['denied_zones']
position_offset = float(data['world_length'] / 2)
color_cursor = 0
colors = [
    'black',
    'red',
    'green',
    'blue',
    'magenta',
    'yellow',
    'cyan',
]

def inDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return True
    return False

def notInDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return False
    return True

def forAll(l):
    for i in l:
        print(i)

def normalPos(p):
    return [p[0] - position_offset, p[1] - position_offset]

def makeClusters():
    global cluster_count
    global cluster_element_treshold
    for i in range(len(special_assets)):
        neighbour_index_list = [i]
        base_point = special_assets[i]
        if not cluster_count:
            for j in range(len(special_assets)):
                if(j != i):
                    d = util.dist(base_point['p'], special_assets[j]['p'])
                    if d <= bridge_length:
                        neighbour_index_list.append(j)
            if len(neighbour_index_list) > cluster_element_treshold:
                cluster_count = cluster_count + 1
                for j in neighbour_index_list:
                    special_assets[j]['c'] = cluster_count
        else:
            if base_point['c']:
                for j in range(len(special_assets)):
                    if(j != i) and (not special_assets[j]['c']):
                        d = util.dist(base_point['p'], special_assets[j]['p'])
                        if d <= bridge_length:
                            special_assets[j]['c'] = base_point['c']
            else:
                for j in range(len(special_assets)):
                    d = util.dist(base_point['p'], special_assets[j]['p'])
                    if d <= bridge_length:
                        if(j != i) and (special_assets[j]['c']):
                            special_assets[i]['c'] = special_assets[j]['c']
                            break
                        neighbour_index_list.append(j)
                if special_assets[i]['c']:
                    continue
                elif len(neighbour_index_list) > cluster_element_treshold:
                    cluster_count += 1
                    for j in neighbour_index_list:
                        special_assets[j]['c'] = cluster_count

for building in data['special_assets']:
    if building['type'] == 'tall_building':
        for p in building['locations']:
            if notInDeniedZone([p]):
                special_assets.append({
                    'p':[
                        float(p[0] + position_offset),
                        float(p[1] + position_offset)
                    ],
                    'c': 0
                })
    else:
        special_assets.append({
            'p':[
                float(building['location']['x'] + position_offset),
                float(building['location']['y'] + position_offset)
            ],
            'c': 0
        })
        special_assets.append({
            'p':[
                float(building['location']['x'] + position_offset),
                float(building['location']['y'] + position_offset)
            ],
            'c': 0
        })

makeClusters()
forAll(special_assets)
print("--- %s seconds ---" % (time.time() - start_time))

for p in special_assets:
    plt.plot(p['p'][0], p['p'][1], marker='.', color=colors[p['c']])
    color_cursor += 1
    color_cursor %= 7
plt.show()
