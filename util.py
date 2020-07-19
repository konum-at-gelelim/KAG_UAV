import math


def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)

def dist2(position1, position2):
    sum = 0
    for i in range(2):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)

def getSin(angle_in_degree):
    return math.sin(math.radians(angle_in_degree))

def getCos(angle_in_degree):
    return math.cos(math.radians(angle_in_degree))

class Point_2d(object):

    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def getList(self):
        return [self.x, self.y]

    def __add__(self, other):
        return type(self)(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return type(self)(self.x - other.x, self.y - other.y)

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y)


class Point_3d(Point_2d):

    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Point_3d, self).__init__(x, y)
        self.z = float(z)

    def getList(self):
        return [self.x, self.y, self.z]

    def __add__(self, other):
        return type(self)(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        return type(self)(self.x - other.x, self.y - other.y, self.z - other.z)

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y) + ' z:' + str(self.z)

class Vector_2d(object):

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def getList(self):
        return [self.x, self.y]

    def getMag(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def getMagnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def getDirection(self):
        return 'void boi'

    def setVectorWithList(self, lelist):
        self.x = lelist[0]
        self.y = lelist[1]

    def setFera(self, max):
        temp = self.getList()
        for i in range(len(temp)):
            if temp[i] < -max:
                temp[i] = -max
            elif temp[i] > max:
                temp[i] = max
        self.setVectorWithList(temp)

    def __add__(self, other):
        return type(self)(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return type(self)(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return float(self.x * other.x + self.y * other.y)

    def __lt__(self, other):
        return self.getMag < other.getMag()

    def __le__(self, other):
        return self.getMag <= other.getMag()

    def __eq__(self, other):
        return self.getMag == other.getMag()

    def __ne__(self, other):
        return self.getMag != other.getMag()

    def __ge__(self, other):
        return self.getMag >= other.getMag()

    def __gt__(self, other):
        return self.getMag() > other.getMag()

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y) + ' mag:' + str(self.getMag())

class Vector_3d(Vector_2d):

    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Vector_3d, self).__init__(x, y)
        self.z = float(z)

    def getList(self):
        return [self.x, self.y, self.z]
    
    def getMag(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def getMagnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def getDirection(self):
        return 'now in 3d, coming sooon'

    def setVectorWithList(self, new_points):
        self.x = new_points[0]
        self.y = new_points[1]
        self.z = new_points[2]

    def setFera(self, max):
        temp = self.getList()
        for i in range(len(temp)):
            if temp[i] < -max:
                temp[i] = -max
            elif temp[i] > max:
                temp[i] = max
        self.setVectorWithList(temp)

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y) + ' z:' + str(self.z) + ' mag:' + str(self.getMag())

def point_2_vector(p):
    return Vector_2d(p.x, p.y)

def point_3_vector(p):
    return Vector_3d(p.x, p.y, p.z)
