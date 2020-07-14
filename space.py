import math

class Point_2d(object):

    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def getList(self):
        return [self.x, self.y]

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y)


class Point_3d(Point_2d):

    def __init__(self, x=0.0, y=0.0, z=0.0):
        super(Point_3d, self).__init__(x, y)
        self.z = float(z)

    def getList(self):
        return [self.x, self.y, self.z]

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y) + ' z:' + str(self.z)

class Vector_2d(object):

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def getMag(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def getMagnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def getDirection(self):
        return 'void boi'

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

    def getMag(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def getMagnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def getDirection(self):
        return 'now in 3d, coming sooon'

    def __str__(self):
        return 'x:' + str(self.x) + ' y:' + str(self.y) + ' z:' + str(self.z) + ' mag:' + str(self.getMag())


#A = Point_2d()
#B = Point_2d(5, 4.5)
#C = Point_3d()
#D = Point_3d(1,2,-3)
#print ""
#print(A)
#print(A.getList())
#print(B)
#print(B.getList())
#print(C)
#print(C.getList())
#print(D)
#print(D.getList())

#V1 = Vector_3d()
#V2 = Vector_3d(1,1,1) 
#print 'vec 1', V1
#print 'vec 2', V2
#print (V1 + V2).getMagnitude()
#print V1 - V2
#print V1 * V2
#print V1 == V2
#print V1 != V2
#print V1.getDirection()

