from time import sleep
from numpy import inf, random
from math import sqrt, sin, cos, atan, pi
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
- getSurroundings()
  After calling this function, the visual thread takes a few pictures of the
  environment and returns a list of potential seen objects. Objects have the
  following format:
    minBottle = {'name': 'bottle', 'position': (0.0, 0.0), 'color': 'yellow'}
    maxBottle = {'name': 'bottle', 'probability': 1.0, 'position': (0.0, 0.0), 'stdev_p': 0.0, 'color': 'yellow'}
    minCone = {'name': 'cone', 'position': (0.0, 0.0)}
    maxCone = {'name': 'cone', 'probability': 1.0, 'position': (0.0, 0.0), 'stdev_p': 0.0}
    minBox = {'name': 'box', 'position': (0.0, 0.0), 'rot': 0.0}
    maxBox = {'name': 'box', 'probability': 1.0, 'position': (0.0, 0.0), 'stdev_p': 0.0, 'rot': 0.0}
  All objects have a parameter name, an optional parameter probability, a
  parameter position, and an optional parameter stdev_p. All bottle objects
  have a parameter color. All box objects have a parameter rot.

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

# debug settings ------------------------------------------------------------- #
DEBUG = True    # print debug
ADVANCED = True # give incorrect input



'''//------------------------------------------------------------------------//'
''// class definitions -----------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# class location, used to store positions on the map ------------------------- #
class Loc(object):
    # class variables
    # x = 0.0
    # y = 0.0

    # constructor
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    # string generator
    def __str__(self):
        return "({:5.1f}, {:5.1f})".format(self.x, self.y)


# class object, used for objects on the map ---------------------------------- #
class Obj(object):
    # class variables
    # prob = 1.0
    # views = 0
    # pos = Loc(0.0, 0.0)
    # pdev = 0.0

    # constructor
    def __init__(self, prob, pos, pdev):
        self.prob = prob
        self.views = 1
        self.pos = pos
        self.pdev = pdev

    # string generator
    def __str__(self):
        return "sort: {}\tprob: {}\tpos: {}\tpdev: {}".format(type(self), self.prob, self.pos, self.pdev)

    # remap object over distance vec, followed by rotation rot over (0.0, 0.0)
    def remap(self, vec, rot):
        newx = self.pos.x * cos(rot) + self.pos.y * sin(rot)
        newy = self.pos.x * -sin(rot) + self.pos.y * cos(rot)
        self.pos = Loc(newx, newy)

    # calculate distance between object and other object
    def getDist(self, obj):
        return sqrt((self.pos.x - obj.pos.x)**2 + (self.pos.y - obj.pos.y)**2)

    # calculate angle between object and other object
    def getAngle(self, obj):
        dx = float(obj.pos.x - self.pos.x)
        dy = float(obj.pos.y - self.pos.y)
        if dx == 0 and dy == 0:
            return 0.0
        elif dy == 0:
            return pi * (dx > 0)
        elif dx == 0:
            return pi * (dy > 0) - pi/2
        elif dy > 0:
            return atan(dx / dy)
        elif dx > 0:
            return atan(dx / dy) + pi
        else:
            return atan(dx / dy) - pi

    # calculate matching error between object and other object
    def matchError(self, obj):
        dst = self.getDist(obj)
        magic = 1 + self.pdev + obj.pdev
        return dst / magic


# class bottle, used to represent the corner bottles of the field -------------#
class Bottle(Obj):
    # class variables
    # color = Enum("Color", "yellow orange blue purple")

    # constructor
    def __init__(self, prob, pos, pdev, color):
        super(Bottle, self).__init__(prob, pos, pdev)
        self.color = color

    # string generator
    def __str__(self):
        return super(Bottle, self).__str__() + "\tcolor: {}".format(self.color)

    # getters/setters
    def getColor(self):
        return self.color


# class cone, used to represent the cones in the field ------------------------#
class Cone(Obj):
    # class variables
    # name = None
    # boxesDone = False

    # constructor
    def __init__(self, prob, pos, pdev):
        super(Cone, self).__init__(prob, pos, pdev)
        self.name = None
        self.boxesDone = False

    # string generator
    def __str__(self):
        return super(Cone, self).__str__() + "\tname: {}\tdone: {}".format(self.name, self.boxesDone)

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
    # rot = 0.0
    # origin = None
    # dest = None
    # delivered = False

    # constructor
    def __init__(self, prob, pos, pdev, rot):
        super(Box, self).__init__(prob, pos, pdev)
        self.rot = rot
        self.origin = None
        self.dest = None
        self.delivered = False

    # remap object over distance vec, followed by rotation rot over (0.0, 0.0)
    def remap(self, vec, rot):
        super(Box, self).remap(vec, rot)
        self.rot += rot

    # string generator
    def __str__(self):
        return super(Box, self).__str__() + "\trot: {:5.3f}\torigin: {}\tdest: {}\tdelivered: {}".format(self.rot, self.origin, self.dest, self.delivered)

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
class Map(object):
    # class variables
    # objs = []

    # constructor
    def __init__(self):
        self.objs = []

    # getters/setters
    def setSize(self, x, y):
        self.width = x
        self.length = y

    # add new objects to the map
    def add(self, obj):
        self.objs.append(obj)

    # find match between objects on map and new object. If no match found, return false
    def findMatch(self, obj):
        minerr = inf
        match = False
        for i in self.objs:
            if type(i) == type(obj):
                err = i.matchError(obj)
                if (err < minerr):
                    minerr = err
                    match = i
        return match

    # duplicates map, with all elements shifted over distance pos, then rotated
    # by rot radians over point (0.0, 0.0)
    def getShiftedCopy(self, pos, rot):
        copy = Map()
        for obj in self.objs:
            tmp = deepcopy(obj)
            tmp.remap(pos, rot)
            copy.add(tmp)
        return copy

    # returns best rotational fit for map B on map A. Arguments: self = map A;
    # mapb = map B, pos = loc shift of map B on map A; rot = rotation finding
    # starting point, rdev = maximum deviation of rotational fit
    def getRotFit(self, mapb, pos, rot, rdev):
        copy = mapb.getShiftedCopy(pos, rot)
        zero = Obj(0.0, Loc(0.0, 0.0), 0.0)
        rdif = []
        for obj in copy.objs:
            match = self.findMatch(obj)
            if match:
                rota = zero.getAngle(match)
                rotb = zero.getAngle(obj)
                rdif.append(rotb-rota)
        if len(rdif):
            return rot + median(rdif)
        else:
            return rot

    # returns best location shift fit for map B on map A. Arguments: self = map
    # A; # mapb = map B, pos = location shift starting point; pdev = maximum
    # location shift; rot = rotational shift of map B on map A
    def getPosFit(self, mapb, pos, pdev, rot):
        print("TODO")
        return Loc(0.0, 0.0)

    # Fit a second map to the first map, return combined map. Starting point of
    # alignment is position (0,0) on map B mapped on map A, rotated by rot
    # radians. Maximum position deviation is fixed to pdev, maximum rotation
    # deviation is fixed to rdev
    def combine(self, mapb, pos, pdev, rot, rdev):
        print("TODO")

    # debug print: print all objects
    def debugPrint(self, nice=False, xx=None, xy=None):
        for obj in self.objs:
            print("{}\t{}".format(self.objs.index(obj), obj))
        if nice:
            xscale = 16
            yscale = 40
            xmin = xmax = 0.0
            ymin = ymax = 0.0
            posx = []
            posy = []
            for obj in self.objs:
                x = obj.pos.x
                y = obj.pos.y
                xmin = min(xmin, x)
                xmax = max(xmax, x)
                ymin = min(ymin, y)
                ymax = max(ymax, y)
                posx.append(x)
                posy.append(y)
            for i in range(0, len(posx)):
                posx[i] = round((posx[i]-xmin)/xscale)
                posy[i] = round((posy[i]-ymin)/yscale)
            for y in range(int(round((ymax-ymin)/yscale)), -1, -1):
                line = "\t"
                for x in range(0, int(round((xmax-xmin)/xscale))+1):
                    match = False
                    for i in range(len(posx)):
                        if posx[i] == x and posy[i] == y:
                            idx = i
                            match = True
                            break
                    if xx != None and xy != None and round((xx-xmin)/xscale) == x and round((xy-ymin)/yscale) == y:
                        line += 'X'
                    elif match:
                        line += str(idx)
                    else:
                        line += '.'
                print(line)



'''//------------------------------------------------------------------------//'
''// test map & vision + Arduino placeholders ------------------------------//''
'//------------------------------------------------------------------------//'''
# create test map ------------------------------------------------------------ #
debugmap = Map()
xmin = 0.0
xmax = random.randint(500, 1000)
ymin = 0.0
ymax = random.randint(700, 1000)
debugmap.add(Bottle(1.0, Loc(xmin, ymin), 0.0, 'yellow'))
debugmap.add(Bottle(1.0, Loc(xmin, ymax), 0.0, 'orange'))
debugmap.add(Bottle(1.0, Loc(xmax, ymax), 0.0, 'blue'))
debugmap.add(Bottle(1.0, Loc(xmax, ymin), 0.0, 'purple'))
names = ['Inaki', 'Oudman', 'Amin']
boxes = 0
for i in range(3):
    x = random.randint(50, xmax-50)
    y = random.randint(50, ymax-50)
    cone = Cone(1.0, Loc(x, y), 0.0)
    cone.setName(names[i])
    debugmap.add(cone)
    n = random.randint(1, 3)
    for j in range(n):
        xp = random.randint(x-25, x+25)
        yp = random.randint(y-25, y+25)
        rot = 2*pi*random.rand()
        dest = random.randint(0,3)
        if dest != i and boxes < 3:
            box = Box(1.0, Loc(xp, yp), 0.0, rot)
            box.setData(names[i], names[dest])
            debugmap.add(box)
            boxes += 1
robotx = random.randint(50, xmax-50)
roboty = random.randint(50, ymax-50)
robotrot = 2*pi*random.rand()
robot = Obj(1.0, Loc(robotx, roboty), 0.0)
if DEBUG:
    print("Debug map created:")
    debugmap.debugPrint(True, robotx, roboty)
    print("Robot placed at position ({}, {}) with rotation {:5.3}\n".format(robotx, roboty, robotrot))


# vision thread placeholder, uses test map ----------------------------------- #
def getSurroundings():
    print("getSurroundings()")
    objects_list = []
    for obj in debugmap.objs:
        dist = robot.getDist(obj)
        angle = (robot.getAngle(obj) - robotrot + pi) % (2*pi) - pi
        if angle > (-pi/3) and angle < (pi/3) and not isinstance(obj, Box):
            if DEBUG:
                print("\tObject in sight? Dist: {:5.1f}, angle: {:5.2f}.".format(dist, angle))
            rnd = random.rand()
            if not ADVANCED or 250/dist >= rnd:
                if ADVANCED:
                    prob = 0.8
                    magic = 25/(dist+25)
                    angle += (magic*random.rand() - magic/2)
                    magic = (dist/1250)**2+0.1
                    dist = (1 + magic*random.rand() - magic/2) * dist
                    pdev = magic*dist/2+10
                else:
                    prob = 1.0
                    pdev = 0.0
                print("\t\tYes, observed at dist {:5.1f} and angle {:5.2f} ({:4.2} > {:4.2})".format(dist, angle, 250/dist, rnd))
                posx = dist * sin(angle)
                posy = dist * cos(angle)
                if isinstance(obj, Bottle):
                    if obj.pos.x == 0.0 and obj.pos.y == 0:
                        color = 'yellow'
                    elif obj.pos.x == 0.0:
                        color = 'orange'
                    elif obj.pos.y == 0.0:
                        color = 'purple'
                    else:
                        color = 'blue'
                    objects_list.append({'name': 'bottle', 'probability': prob, 'position': (posx,posy), 'stdev_p': pdev, 'color': color})
                elif isinstance(obj, Cone):
                    objects_list.append({'name': 'cone', 'probability': prob, 'position': (posx,posy), 'stdev_p': pdev})
            else:
                print("\t\tNope, too cloudy ({:4.2} < {:4.2})".format(250/dist, rnd))
    if DEBUG:
        print("")
    return objects_list


# Arduino placeholder -------------------------------------------------------- #
def getPosition():
    print('Postion requested')
    return Loc(0,0)


# Arduino placeholder, uses test map ----------------------------------------- #
def rotate(angle):
    print('Rotate by {} radians\n'.format(angle))
    global robotrot
    robotrot += angle



'''//------------------------------------------------------------------------//'
''// other functions -------------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# parses input from functional to nice object
def parse(item):
    if('probability' in item):
        prob = item['probability']
    else:
        prob = 1.0
    pos = Loc(item['position'][0], item['position'][1])
    if('stdev_p' in item):
        pdev = item['stdev_p']
    else:
        pdev = 0.0
    if item['name'] == 'bottle':
        color = item['color']
        return Bottle(prob, pos, pdev, color)
    elif item['name'] == 'cone':
        return Cone(prob, pos, pdev)
    elif item['name'] == 'box':
        rot = item['rot']
        return Box(prob, pos, pdev, rot)
    else:
        print("Parsing error")




'''//------------------------------------------------------------------------//'
''// main code, part I: creating initial map -------------------------------//''
'//------------------------------------------------------------------------//'''
# create empty initial map
imap = Map()

# get environment
environment = getSurroundings()

# place objects on map
for i in environment:
    imap.add(parse(i))

if DEBUG:
    print("Initial map created:")
    imap.debugPrint(True, 0, 0)
    print("Robot at (0.0, 0.0) with rotation 0.0\n")

# rotation settings
steps = 12
step = 2 * pi / steps

# look around 360 degrees
rotate(step)
rot = step
while (rot < 2 * pi):
    # get environment
    new = Map()
    environment = getSurroundings()
    for i in environment:
        new.add(parse(i))

    # adjust assumed rotation rot to rot'
    rotp = imap.getRotFit(new, 0.0, rot, step/2)
    print("Observed rotation: {}".format(rotp))
    sleep(10)

    # remap position of observed objects to coordinate system of map
    # TODO

    # update map
    # TODO

    # rotate by step degrees, adjust for rotp
    rotate(step + (rot-rotp))
    rot += step

    if DEBUG:
        print("Updated map")
        imap.debugPrint(True, 0, 0)
        print("Robot at (0.0, 0.0) with rotation {}\n".format(rotp))
        sleep(10)



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
