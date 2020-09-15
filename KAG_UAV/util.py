import math


def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)

def getSin(angle_in_degree):
    return math.sin(math.radians(angle_in_degree))

def getCos(angle_in_degree):
    return math.cos(math.radians(angle_in_degree))

def getMeter(altitude):
    return altitude * 0.3048