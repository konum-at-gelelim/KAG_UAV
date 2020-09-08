import matplotlib.pyplot as plt
import matplotlib.path as mpltPath
import json
import util
import time

with open('param.json') as f:
    data = json.load(f)

class BaseUAV():
    pass

class KagUAV(BaseUAV):
    def __init__(self):
        self.tall_index = 0
        self.special_assets = []
        self.tall_count = None
        self.bridge_length = 100.0
        self.cluster_count = 0
        self.cluster_element_treshold = 3
        self.deniedZones = []
        self.position_offset = 0.0
    
    def assetInDeniedZone(self, p):
        for polygon in self.deniedZones:
            path = mpltPath.Path(polygon)
            if path.contains_points(p):
                return True
        return False
    
    def assetNotInDeniedZone(self, p):
        for polygon in self.deniedZones:
            path = mpltPath.Path(polygon)
            if path.contains_points(p):
                return False
        return True
    
    def makeClusters(self):
        for i in range(len(self.special_assets)):
            neighbour_index_list = [i]
            base_point = self.special_assets[i]
            if not self.cluster_count:
                for j in range(len(self.special_assets)):
                    if(j != i):
                        d = util.dist(base_point['p'], self.special_assets[j]['p'])
                        if d <= self.bridge_length:
                            neighbour_index_list.append(j)
                if len(neighbour_index_list) > self.cluster_element_treshold:
                    self.cluster_count = self.cluster_count + 1
                    for j in neighbour_index_list:
                        self.special_assets[j]['c'] = self.cluster_count

            else:
                if base_point['c']:
                    for j in range(len(self.special_assets)):
                        if(j != i) and (not self.special_assets[j]['c']):
                            d = util.dist(base_point['p'], self.special_assets[j]['p'])
                            if d <= self.bridge_length:
                                self.special_assets[j]['c'] = base_point['c']
                else:
                    for j in range(len(self.special_assets)):
                        d = util.dist(base_point['p'], self.special_assets[j]['p'])
                        if d <= self.bridge_length:
                            if(j != i) and (self.special_assets[j]['c']):
                                self.special_assets[i]['c'] = self.special_assets[j]['c']
                                break
                            neighbour_index_list.append(j)
                    if self.special_assets[i]['c']:
                        continue
                    elif len(neighbour_index_list) > self.cluster_element_treshold:
                        self.cluster_count += 1
                        for j in neighbour_index_list:
                            self.special_assets[j]['c'] = self.cluster_count
             
    def cluster_time(self):
        while(data['special_assets'][self.tall_index]['type'] != 'tall_building'):
            self.tall_index += 1
        self.tall_count = len(data['special_assets'][self.tall_index]['locations'])
        self.deniedZones = data['denied_zones']
        self.position_offset = float(data['world_length'] / 2)
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
        for building in data['special_assets']:
            if building['type'] == 'tall_building':
                for p in building['locations']:
                    if self.assetNotInDeniedZone([p]):
                        self.special_assets.append({
                            'p':[
                                float(p[0] + self.position_offset),
                                float(p[1] + self.position_offset)
                            ],
                            'c': 0
                        })
            else:
                self.special_assets.append({
                    'p':[
                        float(building['location']['x'] + self.position_offset),
                        float(building['location']['y'] + self.position_offset)
                    ],
                    'c': 0
                })
                self.special_assets.append({
                    'p':[
                        float(building['location']['x'] + self.position_offset),
                        float(building['location']['y'] + self.position_offset)
                    ],
                    'c': 0
                })
        self.makeClusters()
        util.forAll(self.special_assets)
        for p in self.special_assets:
            plt.plot(p['p'][0], p['p'][1], marker='.', color=colors[p['c']])
            color_cursor += 1
            color_cursor %= 7
        plt.show()

kek = KagUAV()
kek.cluster_time()