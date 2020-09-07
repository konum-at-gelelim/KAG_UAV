import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import json
import util
import time

#BURAYI OKUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU
# ! param.json benim json dosyası !
# ! projedeki util dosyasında dist() fonksiyonu var onu kullanıyor bu !
# ! bu kadar, ted konuşmamı okuduğun için eyw !

start_time = time.time()
with open('param.json') as f:
    data = json.load(f)

cluster_count = 1
cluster_min_element = 4
tall_index = 0
while(data['special_assets'][tall_index]['type'] != 'tall_building'):
    tall_index += 1
tall_buildings = []
tall_buildings_adapted = []
tall_count = len(data['special_assets'][tall_index]['locations'])
hospitals = []
hospitals_adapted = []

if data['special_assets'][tall_index]['width'][0] > data['special_assets'][tall_index]['width'][1]:
    bridge_length = data['special_assets'][tall_index]['width'][0] * 2.5
else:
    bridge_length = data['special_assets'][tall_index]['width'][1] * 2.5
deniedZones = data['denied_zones']
position_offset = float(data['world_length'] / 2)

def inDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return True
    return False

for zone in data['special_assets']:
    if zone['type'] == 'tall_building':
        for t in zone['locations']:
            tall_buildings.append(t)
    else:
        hospitals.append([
            zone['location']['x'],
            zone['location']['y']
            ])

def forAll(l):
    for i in l:
        print(i)

def normalPos(p):
    return [p[0] - position_offset, p[1] - position_offset]

def get_neighbour_count(i):
    global cluster_count
    c = 0
    p = tall_buildings_adapted[i]['p']
    neighbourList = [i]
    for j in range(len(tall_buildings_adapted)):
        if (j != i) and (tall_buildings_adapted[j]['c'] == 0):
            t = tall_buildings_adapted[j]['p']
            d = util.dist(p, t)
            if d <= bridge_length:
                c += 1
                neighbourList.append(j)
    if c >= cluster_min_element:
        for j in neighbourList:
            tall_buildings_adapted[j]['c'] = cluster_count
        cluster_count += 1



while(len(tall_buildings) != 0):
    temp = [position_offset * 2 + 1, -1.0]
    pop_index = None
    for i in range(len(tall_buildings)):
        next_tall = [float(tall_buildings[i][0] + position_offset), float(tall_buildings[i][1] + position_offset)]
        if (next_tall[1] > temp[1]) or (next_tall[1] == temp[1] and next_tall[0] < temp[0]):
            temp[0] = float(next_tall[0])
            temp[1] = float(next_tall[1])
            pop_index = i
    tall_buildings_adapted.append({
        'p': temp,
        'c': 0
        }) 
    del(tall_buildings[pop_index])

for i in range(len(tall_buildings_adapted)):
    get_neighbour_count(i)
forAll(tall_buildings_adapted)
print(len(tall_buildings_adapted))
print("--- %s seconds ---" % (time.time() - start_time))
