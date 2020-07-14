import math


def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)