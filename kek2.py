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
special_assets = []
tall_count = len(data['special_assets'][tall_index]['locations'])

if data['special_assets'][tall_index]['width'][0] > data['special_assets'][tall_index]['width'][1]:
    bridge_length = data['special_assets'][tall_index]['width'][0] * 2.5
else:
    bridge_length = data['special_assets'][tall_index]['width'][1] * 2.5

deniedZones = data['denied_zones']
position_offset = float(data['world_length'] / 2)
colors = [
    'blue',
    'green',
    'red',
    'cyan',
    'magenta',
    'yellow',
    'black',
    'white'
]

def inDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return True
    return False

def get_neighbour_count(i):
    pass

def forAll(l):
    for i in l:
        print(i)

def normalPos(p):
    return [p[0] - position_offset, p[1] - position_offset]


for building in data['special_assets']:
    if building['type'] == 'tall_building':
        for p in building['locations']:
            special_assets.append([
                float(p[0] + position_offset),
                float(p[1] + position_offset)
            ])
    else:
        special_assets.append([
            float(building['location']['x'] + position_offset),
            float(building['location']['y'] + position_offset)
            ])
        special_assets.append([
            float(building['location']['x'] + position_offset),
            float(building['location']['y'] + position_offset)
            ])

for p in special_assets:
    plt.plot(p[0], p[1], marker='.', color='black')
plt.show()

print("--- %s seconds ---" % (time.time() - start_time))
print
