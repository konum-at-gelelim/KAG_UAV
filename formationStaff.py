from math import sin, cos, radians
from time import sleep
import matplotlib.pyplot as plt

def rotateUndTraslate(formation_array, angle, pivot):
    for i in range(len(formation_array)):
        sin_value = sin(radians(angle))
        cos_value = cos(radians(angle))
        p = formation_array[i]
        formation_array[i] = [
            (p[0] * cos_value - p[1] * sin_value) + pivot[0],
            (p[0] * sin_value + p[1] * cos_value) + pivot[1],
            p[2]
        ]
    return formation_array

def prism_gen(uav_location, a_k, u_b, u_k, uav_count):
    prism_formation = []
    pivot = [uav_location[0], uav_location[1]]
    a_k = (a_k - 90) % 360
    x = 0.0 - u_k
    y = 0
    z = uav_location[2]
    prism_formation.append([
        x,
        y,
        z
    ])
    row_multiplier = 1
    row_node = 1
    while(len(prism_formation) < uav_count):
        x = prism_formation[0][0] - u_b * row_multiplier
        if row_node % 2 == 1:
            y = prism_formation[0][1] + u_b / 2.0
        else:
            y = prism_formation[0][1] - u_b / 2.0
        if row_node < 3:
            z = prism_formation[0][2] + u_b / 2.0
        else:
            z = prism_formation[0][2] - u_b / 2.0
            if row_node == 4:
                row_node = 0
                row_multiplier = row_multiplier + 1
        prism_formation.append([
            x,
            y,
            z
        ])
        row_node = row_node + 1

    prism_formation = rotateUndTraslate(prism_formation, a_k, pivot)
    return prism_formation

def arrow_head_gen(uav_location, a_b, a_k, u_b, u_k, uav_count):
    arrow_formation = []
    pivot = [uav_location[0], uav_location[1]]
    a_k = (a_k - 90) % 360
    x = 0.0 - u_k
    y = 0.0
    arrow_formation.append([
        x,
        y,
        uav_location[2]
    ])
    left_wing_angle = (180.0 - a_b) % 360.0
    left_sin_value = sin(radians(left_wing_angle))
    left_cos_value = cos(radians(left_wing_angle))
    right_wing_angle = (180.0 + a_b) % 360.0
    right_sin_value = sin(radians(right_wing_angle))
    right_cos_value = cos(radians(right_wing_angle))
    row_multiplier = 1
    next_is_left = True
    while(len(arrow_formation) < uav_count):
        if next_is_left:
            x = arrow_formation[0][0] + u_b * left_cos_value * row_multiplier
            y = arrow_formation[0][1] + u_b * left_sin_value * row_multiplier
        else:
            x = arrow_formation[0][0] + u_b * right_cos_value * row_multiplier
            y = arrow_formation[0][1] + u_b * right_sin_value * row_multiplier
            row_multiplier = row_multiplier + 1
        next_is_left = not next_is_left
        arrow_formation.append([x, y, altitude])

    arrow_formation = rotateUndTraslate(arrow_formation, a_k, pivot)
    return arrow_formation

#---------------------- MAIN ------------------------------#

uav_count = 27
uav_location = [3000.0, -365.0, 100.0]
uav_location2 = [3000.0, 365.0, 100]
altitude = 100.0
u_k = 50
u_b = 40
a_b = 45
a_k = -90.952


ok = arrow_head_gen(uav_location, a_b, a_k, u_b, u_k, uav_count)
kutu = prism_gen(uav_location2, a_k, u_b, u_k, uav_count)

#ok plot ekleyici
for e in ok:
    print e
    plt.plot(e[0], e[1], 'ro')

#kutu plot ekleyici
#for e in kutu:
#    print e
#    plt.plot(e[0], e[1], 'ro')

#plt.show(block=False)
#plt.pause(1)
#plt.close()

plt.show()

