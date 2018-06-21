from time import sleep
from numpy import inf
from math import sqrt, sin, cos, atan
from copy import deepcopy
from statistics import median
from enum import Enum

'''
tasks of control thread:
- receive estimated position from Arduino
- send move instructions to Arduino
- send grip/drop instructions to Arduino
- receive surroundings interpretations from visual thread
- keep track of map of surroundings
- keep track of own position on map
- keep track of status of persons and boxes

Assumed functions in visual thread:
- list[object, probability, position, stdev, rotation, stdev] getSurroundings()
  After calling this function, the visual thread takes a few pictures of the
  environment and returns a list of potential seen objects. Each object has a 
  probability of being that specific object, an estimated position and standard
  deviation and an estimated rotation and standard deviation. These last two
  parameters are only useful for boxes, not for cones and bottles.

Assumed functions in Arduino:
- position getPosition()
  Returns estimated position relative to position at last function call.
- void rotate(angle)
  Rotate on center between wheels over given angle
- void move(alpha, distance)
  Move to new position at angle alpha at distance distance
- void move(alpha, distance, beta)
  Move to new position at angle alpha at distance distance, end up at rotation
  beta
- grab(gnum)
  Open gripper gnum if necessary, lower gripper arm, close gripper gnum, raise 
  gripper arm
- drop(gnum)
  Lower gripper arm, open gripper gnum, raise gripper arm
'''



'''//------------------------------------------------------------------------//'
''// temporary placeholders ------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# temporary Vision placeholder ----------------------------------------------- #
def getSurroundings():
    Num_of_objects = 2
    object_properties = {'name': 'bottle', 'probability': 0.0, 'position': (0,0), 'stdev_p': 0, 'rotation': 0.0 , 'stdev_r': 0.0}
    objects_list = [object_properties for _ in range(Num_of_objects)]
    return objects_list

# temporary Arduino placeholders --------------------------------------------- #
def getPosition():
    print('Postion requested')
    return Loc(0,0)

def rotate(angle):
    print('Rotate {} degrees'.format(angle))



'''//------------------------------------------------------------------------//'
''// class definitions -----------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# class location, used to store positions on the map ------------------------- #
class Loc:
    # class variables
    x = 0.0
    y = 0.0
    
    # constructor
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    # return string representation of loc
    def getString(self):
        return "({},{})".format(self.x, self.y)


# class object, used for objects on the map ---------------------------------- #
class Obj:
    # class variables
    sort = Enum("Type", "Cone Bottle Box")
    prob = 1.0
    pos = 0.0
    pdev = 0.0
    
    # constructor
    def __init__(self, item):
        self.sort = item['name']
        if('probability' in item):
            self.prob = item['probability']
        else:
            self.prob = 1.0
        self.pos = Loc(item['position'][0], item['position'][1]) 
        if('stdev_p' in item):
            self.pdev = item['stdev_p']
        else:
            self.pdev = 0.0;
    
    # remap object over distance vec, followed by rotation rot
    def remap(self, vec, rot):
        self.pos += vec
        self.rot += rot
    
    # remap object as if origin was pos instead of (0,0) and if angle was rot instead of 0
    def inverseremap(self, pos, angle):
        self.pos -= pos
        newx = self.pos.x * cos(angle) + self.pos.y * sin(angle) 
        newy = self.pos.x * -sin(angle) + self.pos.y * cos(angle)
        self.pos = Loc(newx, newy)
        self.rot -= angle
    
    # calculate distance between object and other object
    def dist(self, obj):
        return sqrt((self.pos.x - obj.pos.x)**2 + (self.pos.y - obj.pos.y)**2)
    
    # calculate matching error between object and other object
    def matchError(self, obj):
        dst = self.dist(obj)
        magic = 1 + self.pdev + obj.pdev
        return dst / magic
    
    # debug print: print object variables
    def debug(self):
        print("sort: {}\tprob: {}\tpos: {}\tpdev: {}".format(self.sort, self.prob, self.pos.getString(), self.pdev))


# class bottle, used to represent the corner bottles of the field -------------#
class Bottle(Obj):
    # class variables
    color = Enum("Color", "yellow orange blue purple")
    
    # constructor
    def __init__(self, color):
        self.color = color
    
    # getters/setters
    def getColor(self):
        return self.color


# class cone, used to represent the cones in the field ------------------------#
class Cone(Obj):
    # class variables
    name = None
    boxesDone = False
    
    # constructor
    def __init__(self):
        self.name = None
        self.boxesDone = False
    
    # getters/setters
    def getName(self):
        return self.name
    
    def setName(self, name):
        self.name = name
    
    def setBoxesDone(self):
        self.boxesDone = True
    
    # returns true if the robot still has to visit the cone
    def toVisit(self):
        return self.name == None or not boxesDone


# class box, used to represent the boxes in the field -------------------------#
class Box(Obj):
    # class variables
    rot = 0.0
    origin = None
    dest = None
    delivered = False
    
    # constructor
    def __init__(self, rot):
        self.rot = rot
    
    # getters/setters
    def getRot(self):
        return self.rot
    
    def getDest(self):
        return self.dest
    
    def isDelivered(self):
        return self.delivered
    
    def setData(self, origin, dest):
        self.origin = origin
        self.dest = dest
        self.delivered = False
    
    def setDelivered(self):
        self.delivered = True


# class map, our virtual map of the surroundings ----------------------------- #
class Map:
    # class variables
    objs = []
    
    # constructor
    def __init__(self):
        print "new map created, yay"
    
    # find match between objects on map and new object. If no match found, return false
    def findMatch(self, obj):
        minerr = inf
        match = False
        for i in objs:
            if i.sort == obj.sort:
                err = matchError(i, obj)
                if (err < minerr):
                    minerr = err
                    match = i
        return match
    
    # add new objects to the map
    def add(self, obj):
        self.objs.append(obj)
    
    # debug print: print all objects
    def debug(self):
        print("Objs:")
        for obj in self.objs:
            obj.debug()



'''//------------------------------------------------------------------------//'
''// other functions -------------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# return best fitting rotation for objects objs on map imap in range (rot-dev) to (rot+dev)
def imapRotFit(imap, rot, dev, objs):
    rotp = []
    for obj in objs:
        print obj.pos
        dst = sqrt(obj.pos.x**2 + obj.pos.y**2)
        if (dst > 0):
            alp = atan(obj.pos.x / obj.pos.y)
            match = imap.findMatch({'name': obj.sort, 'position': Loc(dst * sin(alp+rot), dst * cos(alp+rot))})
            if(match == False):
                print("No match found :(")
            else:
                deg = atan(match.pos.x / match.pos.y) - rot
                print("Match found, {} degrees off")
                if(abs(deg) < dev):
                    rotp.append(deg)
    return median(rotp)



'''//------------------------------------------------------------------------//'
''// main code, part 0: creating debug map -------------------------------//''
'//------------------------------------------------------------------------//'''
# TODO



'''//------------------------------------------------------------------------//'
''// main code, part I: creating initial map -------------------------------//''
'//------------------------------------------------------------------------//'''
# create empty initial map
imap = Map()

# get environment
environment = getSurroundings()

# place objects on map
for i in environment:
    imap.add(Obj(i))

print("debug stuff:")
imap.debug()
sleep(10)
print("continueing")

# rotation settings
steps = 12
step = 360 / steps

# look around 360 degrees
rotate(step)
for rot in range(step, 360, step):
    # get environment
    objs = []
    environment = getSurroundings()
    for i in environment:
        objs.append(Obj(i))
    
    # adjust assumed rotation rot to rot'
    rotp = imapRotFit(imap, rot, step/2, objs)
    
    # remap position of observed objects to coordinate system of map
    for obj in objs:
        obj.inverseremap(0.0, rotp)
    
    
    # update map
    # TODO
    
    # rotate by step degrees, adjust for rotp
    rotate(step + (rot-rotp))



'''//------------------------------------------------------------------------//'
''// main code, part II: remapping to final map with more features, yay ----//''
'//------------------------------------------------------------------------//'''
# remap items on map to a new rectangular map using the given corner bottle order
# TODO

# make a list of cone statuses
# TODO

# make an empty list of boxes held by the robot
# TODO

'''
At this point we have a nice map containing the following:
- exactly four corner bottles
- a map border
- our own position and rotation
- some, maybe not all, cones

Also, we have a list of cone statuses, containing per cone:
- name of person
- indication if all boxes have been delivered

Also, we have a list of boxes held by the robot, having per box
- gripper number
- delivery address
'''



'''//------------------------------------------------------------------------//'
''// main code, part III: driving around, picking up bitches and boxes -----//''
'//------------------------------------------------------------------------//'''
# while there are cones with undelivered boxes next to it
    # if gripper available
        # move to nearest delivery address OR nearest cone with undelivered boxes
    # else
        # move to nearest delivery address
    # if at delivery address
        # drop box
    # if at cone with undelivered boxes
        # pickup box        



'''//------------------------------------------------------------------------//'
''// main code, part IV: done ----------------------------------------------//''
'//------------------------------------------------------------------------//'''
print("Done. Yay. Beer?")
