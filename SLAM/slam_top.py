from time import sleep
from numpy import inf, random
from math import sqrt, sin, cos, atan, pi
from copy import deepcopy
from statistics import median
from enum import Enum
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from vision import *
import serial

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
    maxBottle = {'name': 'bottle', 'probability': 1.0, 'position': (0.0, 0.0), 'stdev_p': 0.0, 'rot': 0.0, 'stdev_r': 0.0, 'color': 'yellow'}
    minCone = {'name': 'cone', 'position': (0.0, 0.0)}
    maxCone = {'name': 'cone', 'probability': 1.0, 'position': (0.0, 0.0), 'stdev_p': 0.0, 'rot': 0.0, 'stdev_r': 0.0}
    minBox = {'name': 'box', 'position': (0.0, 0.0), 'rot': 0.0}
    maxBox = {'name': 'box', 'probability': 1.0, 'position': (0.0, 0.0), 'stdev_p': 0.0, 'rot': 0.0, 'stdev_r': 0.0}
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
DEBUG = 3       # debug print level: 0 = off, 1 = basic, 2 = medium, 3 = all
ADVANCED = True # give incorrect input
XSCALE = 32     # a terminal character width represents XSCALE centimeters 
YSCALE = 68     # a terminal character height represents YSCALE centimeters



'''//------------------------------------------------------------------------//'
''// class definitions -----------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# class location, used to store positions on the map ------------------------- #
class Loc(object):
    # class variables
    # x = 0.0
    # y = 0.0
    # pdev = 0.0
    # rot = 0.0
    # rdev = 0.0

    # constructor
    def __init__(self, x, y, pdev=None, rot=None, rdev=None):
        self.x = float(x)
        self.y = float(y)
        self.pdev = pdev
        self.rot = rot
        self.rdev = rdev

    # string generator
    def __str__(self):
        str = "({:5.1f}, {:5.1f})".format(self.x, self.y, self.pdev)
        if self.pdev:
            str += " +/- {:4.1f}".format(self.pdev)
        if self.rot != None:
            str += "\trot: {:5.3f}".format(self.rot)
        if self.rdev:
            str += " +/- {:5.3f}".format(self.rdev)
        return str

    # calculate distance between loc and self
    def getDist(self, loc):
        return sqrt((self.x - loc.x)**2 + (self.y - loc.y)**2)

    # calculate angle between loc and self
    def getAngle(self, loc=None):
        if loc == None:
            dx = self.x
            dy = self.y
        else:
            dx = self.x - loc.x
            dy = self.y - loc.y
        if dx == 0 and dy == 0:
            return 0.0
        elif dx == 0:
            return pi * (dy < 0)
        elif dy == 0:
            return pi * (dx > 0) - pi/2
        elif dy > 0:
            return atan(float(dx) / dy)
        elif dx > 0:
            return atan(float(dx) / dy) + pi
        else:
            return atan(float(dx) / dy) - pi

    # take weighted average of two locations
    def wavg(self, loc, w):
        iw = 1 - w
        wx = w*self.x + iw*loc.x
        wy = w*self.y + iw*loc.y
        if self.pdev != None and loc.pdev != None:
            wpdev = float(pow(self.pdev,w)*pow(loc.pdev,iw))/sqrt(2)
        elif self.pdev != None:
            wpdev = self.pdev
        else:
            wpdev = loc.pdev
        if self.rot != None and loc.rot != None:
            rota = self.rot % (2*pi)
            rotb = loc.rot % (2*pi)
            if rota-rotb > pi:
                rota -= 2*pi
            elif rotb-rota > pi:
                rotb -= 2*pi
            wrot = w*rota + iw*rotb
        elif self.pdev != None:
            wrot = self.rot
        else:
            wrot = loc.rot
        if self.rdev != None and loc.rdev != None:
            wrdev = float(pow(self.rdev,w)*pow(loc.rdev,iw))/sqrt(2)
        elif self.pdev != None:
            wrdev = self.rdev
        else:
            wrdev = loc.rdev
        return Loc(wx, wy, wpdev, wrot, wrdev)

    def matchError(self, loc):
        dst = self.getDist(loc)
        magic = 1 + self.pdev + loc.pdev # TODO: tweak this formula
        return float(dst)/magic


# class object, used for objects on the map ---------------------------------- #
class Obj(object):
    # class variables
    # prob = 1.0
    # views = 0
    # pos = Loc(0.0, 0.0, 0.0)

    # constructor
    def __init__(self, prob, pos):
        self.prob = prob
        self.views = 1
        self.pos = pos

    # string generator
    def __str__(self):
        return "sort: {}\tprob: {}\tviews: {}\tpos: {}".format(type(self), self.prob, self.views, self.pos)

    # calculate distance between object and other object
    def getDist(self, obj=None):
        if obj==None:
            return self.pos.getDist(Loc(0.0, 0.0))
        else:
            return self.pos.getDist(obj.pos)

    # calculate angle between object and other object
    def getAngle(self, obj=None):
        if obj==None:
            return self.pos.getAngle(Loc(0.0, 0.0))
        else:
            return self.pos.getAngle(obj.pos)

    # remap object over distance (vec.x, vec.y), followed by rotation vec.rot
    # over (0.0, 0.0)
    def remap(self, vec):
        newx = self.pos.x * cos(vec.rot) + self.pos.y * sin(vec.rot)
        newy = self.pos.x * -sin(vec.rot) + self.pos.y * cos(vec.rot)
        newrot = None
        if self.pos.rot != None:
            newrot = self.pos.rot + vec.rot
        self.pos = Loc(newx, newy, self.pos.pdev, newrot, self.pos.rdev)

    # calculate matching error between object and other object
    def matchError(self, obj):
        if type(self) != type(self):
            return inf
        if isinstance(self, Bottle) and self.color != obj.color:
            return inf
        if isinstance(self, Cone) and self.name != None and obj.name != None and self.name != obj.name:
            return inf
        return self.pos.matchError(obj.pos)

    # update self with object obj
    def update(self, obj):
        weight_prob = 0.5 # TODO: do stuff with the prob parameter
        weight_views = float(self.views) / (self.views+obj.views)
        weight_pdev = 1 - float(self.pos.pdev) / (self.pos.pdev+obj.pos.pdev)
        weight_total = 0.0*weight_prob + 0.7*weight_views + 0.3*weight_pdev # TODO: tweak these values
        if DEBUG >= 3:
            print("w1: {:5.3f}, w2: {:5.3f}, w3: {:5.3f}, wt: {:5.3f}".format(weight_prob, weight_views, weight_pdev, weight_total))
        self.pos = self.pos.wavg(obj.pos, weight_total)
        self.views += obj.views


# class bottle, used to represent the corner bottles of the field -------------#
class Bottle(Obj):
    # class variables
    # color = Enum("Color", "yellow orange blue purple")

    # constructor
    def __init__(self, prob, pos, color):
        super(Bottle, self).__init__(prob, pos)
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
    def __init__(self, prob, pos):
        super(Cone, self).__init__(prob, pos)
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

    # update self with object obj
    def update(self, obj):
        super(Cone, self).update(obj)
        if self.name == None:
            self.name = obj.name
        self.boxesDone = self.boxesDone or obj.boxesDone


# class box, used to represent the boxes in the field -------------------------#
class Box(Obj):
    # class variables
    # origin = None
    # dest = None
    # delivered = False

    # constructor
    def __init__(self, prob, pos):
        super(Box, self).__init__(prob, pos)
        self.origin = None
        self.dest = None
        self.delivered = False

    # string generator
    def __str__(self):
        return super(Box, self).__str__() + "\torigin: {}\tdest: {}\tdelivered: {}".format(self.origin, self.dest, self.delivered)

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

    # update self with object obj
    def update(self, obj):
        super(Box, self).update(obj)
        if self.origin == None:
            self.origin = obj.origin
        if self.dest == None:
            self.dest = obj.dest
        self.delivered = self.delivered or obj.delivered


# class map, our virtual map of the surroundings ----------------------------- #
class Map(object):
    # class variables
    # objs = []
    # locs = []

    # constructor
    def __init__(self):
        self.objs = []
        self.locs = [] # TODO: do use this variable

    # getters/setters
    def getBottle(self, color):
        for obj in self.objs:
            if isinstance(obj, Bottle) and obj.color == color:
                return obj # TODO: fix for situation where we have seen multiple bottles of the same color
        return False

    def setSize(self, x, y):
        self.width = x
        self.length = y

    # add new objects to the map
    def add(self, obj):
        self.objs.append(obj)

    # duplicates map, with all elements shifted over distance (vec.x, vec.y),
    # then rotated by vec.rot radians over point (0.0, 0.0)
    def getShiftedCopy(self, vec):
        copy = Map()
        for obj in self.objs:
            tmp = deepcopy(obj)
            tmp.remap(vec)
            copy.add(tmp)
        return copy

    # find match between objects on map and new object. If no match found,
    # return false
    def findMatch(self, obj):
        minerr = 1 # TODO: tweak this value
        match = False
        for i in self.objs:
            if type(i) == type(obj):
                err = i.matchError(obj)
                if (err < minerr):
                    minerr = err
                    match = i
        return match

    # calculate and return matching error between self and mapb
    def matchError(self, mapb):
        error = 0
        for obj in mapb.objs:
            match = self.findMatch(obj)
            if match:
                error += match.matchError(obj)
            else:
                error += 100 # TODO: tweak this value
        return error

    # returns best rotational fit for map B on map A. Arguments: self = map A;
    # mapb = map B, (vec.x, vec.y) = location shift, vec.rot = rotation
    # starting point, vec.rdev = maximum rotation deviation
    def getRotFit(self, mapb, vec):
        copy = mapb.getShiftedCopy(vec)
        rdif = []
        for obj in copy.objs:
            match = self.findMatch(obj)
            if match:
                rota = obj.pos.getAngle()
                rotb = match.pos.getAngle()
                drot = (rotb-rota+pi) % (2*pi) - pi
                if DEBUG >= 3:
                    print("rota: {:5.3}\trotb: {:5.3}\tdrot: {:5.3}".format(rota, rotb, drot))
                if abs(drot) < vec.rdev:
                    rdif.append(drot)
        if len(rdif):
            return vec.rot + median(rdif)
        else:
            return vec.rot

    # returns best location shift fit for map B on map A. Arguments: self = map
    # A; # mapb = map B, (vec.x, vec.y) = location shift starting point,
    # vec.pdev = maximum shift deviation, vec.rot = rotation shift
    def getPosFit(self, mapb, vec):
        print("TODO") # TODO: implement
        return Loc(0.0, 0.0, 100.0, 0.0, pi)

    # returns best rotation + location shift fit for map B on map A. Arguments:
    # self = map A; # mapb = map B, (vec.x, vec.y) = location shift starting
    # point, vec.pdev = maximum shift deviation, vec.rot = rotation starting
    # point, vec.rdev = maximum rotation deviation
    def getRotPosFit(self, mapb, vec):
        print("TODO") # TODO: implement

    # Update map with objects from given view
    def update(self, view):
        for obj in view.objs:
            match = self.findMatch(obj)
            if match:
                match.update(obj)
            else:
                self.add(obj)

    # debug print: print all objects
    def debugPrint(self, nice=False, xx=None, xy=None):
        for obj in self.objs:
            print("{}\t{}".format(self.objs.index(obj), obj))
        if nice:
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
                posx[i] = round(float(posx[i]-xmin)/XSCALE)
                posy[i] = round(float(posy[i]-ymin)/YSCALE)
            for y in range(int(round(float(ymax-ymin)/YSCALE)), -1, -1):
                line = "\t"
                for x in range(0, int(round(float(xmax-xmin)/XSCALE))+1):
                    match = False
                    for i in range(len(posx)):
                        if posx[i] == x and posy[i] == y:
                            idx = i
                            match = True
                            break
                    if xx != None and xy != None and round(float(xx-xmin)/XSCALE) == x and round(float(xy-ymin)/YSCALE) == y:
                        line += 'X'
                    elif match:
                        line += str(idx)
                    else:
                        line += '-'
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
debugmap.add(Bottle(1.0, Loc(xmin, ymin, 0.0), 'yellow'))
debugmap.add(Bottle(1.0, Loc(xmin, ymax, 0.0), 'orange'))
debugmap.add(Bottle(1.0, Loc(xmax, ymax, 0.0), 'blue'))
debugmap.add(Bottle(1.0, Loc(xmax, ymin, 0.0), 'purple'))
names = ['Inaki', 'Oudman', 'Amin']
boxes = 0
for i in range(3):
    good = False
    while not good:
        x = random.randint(50, xmax-50)
        y = random.randint(50, ymax-50)
        good = True
        for obj in debugmap.objs:
            if obj.pos.getDist(Loc(x, y)) < 100:
                good = False
    cone = Cone(1.0, Loc(x, y, 0.0))
    cone.setName(names[i])
    debugmap.add(cone)
    n = random.randint(1, 3)
    for j in range(n):
        xp = random.randint(x-25, x+25)
        yp = random.randint(y-25, y+25)
        rot = 2*pi*random.rand()
        dest = random.randint(0,3)
        if dest != i and boxes < 3:
            box = Box(1.0, Loc(xp, yp, 0.0, rot, 0.0))
            box.setData(names[i], names[dest])
            debugmap.add(box)
            boxes += 1
robotx = random.randint(50, xmax-50)
roboty = random.randint(50, ymax-50)
robotrot = 2*pi*random.rand()
robot = Obj(1.0, Loc(robotx, roboty, 0.0))
if DEBUG:
    print("Debug map created:")
    debugmap.debugPrint(True, robotx, roboty)
    print("Robot placed at position ({}, {}) with rotation {:5.3f}\n".format(robotx, roboty, robotrot))


# vision thread placeholder, uses test map ----------------------------------- #
def getSurroundings():
    objects_list = Object_detector_inst.objects_process();
    yn = raw_input('Save the image? y/n ')
    print "getSurroundings\n"
    print "========================================================\n"
    print objects_list
    print "========================================================\n"
    print "\n\n\n"
    return objects_list


# Arduino placeholder -------------------------------------------------------- #
def getPosition():
    print('Postion requested')
    return Loc(0,0)


# Arduino placeholder, uses test map ----------------------------------------- #
def rotate(angle):
    #if DEBUG >= 2:
    #   print('Rotate by {} radians\n'.format(angle))
    #  sleep(1)
    ser_msg = 'rotate('+str(angle/2)+')'
    ser.write(ser_msg)
    while(True):
        recv = ser.read(2)
        print recv
        if (recv == 'OK'):
            break
	print 'waiting for Arduino... \n'
    print 'Arduino done... \n'
    global robotrot
    robotrot += angle



'''//------------------------------------------------------------------------//'
''// other functions -------------------------------------------------------//''
'//------------------------------------------------------------------------//'''
# parses input from functional to nice object
def parse(item):
    prob = 1.0 # TODO: fix this unwanted situation
    if('probability' in item):
        prob = item['probability']
    x = item['position'][0]
    y = item['position'][1]
    pdev = 100.0 # TODO: fix this unwanted situation
    if('stdev_p' in item):
        pdev = item['stdev_p']
    rot = None
    if('rot' in item):
        rot = item['rot']
    rdev = None
    if('stdev_r' in item):
        rot = item['stdev_r']
    pos = Loc(x, y, pdev, rot, rdev)
    if item['name'] == 'bottle':
        color = item['color']
        return Bottle(prob, pos, color)
    elif item['name'] == 'cone':
        return Cone(prob, pos)
    elif item['name'] == 'box':
        return Box(prob, pos)
    else:
        print("Parsing error")



'''//------------------------------------------------------------------------//'
''// main code, part I: creating initial map -------------------------------//''
'//------------------------------------------------------------------------//'''
print("+------------------------------------------------------------------------------+")
print("| STAGE I STARTING                                                             |")
print("+------------------------------------------------------------------------------+\n")

# create empty initial map
Object_detector_inst = Object_detector(1)
ser = serial.Serial('/dev/ttyUSB0', 9600)
imap = Map()

# get environment
environment = getSurroundings()

# place objects on map
for i in environment:
    imap.add(parse(i))

if DEBUG >= 2:
    print("Initial view, starting map:")
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
    view = Map()
    environment = getSurroundings()
    for i in environment:
        view.add(parse(i))

    if DEBUG >= 3:
        print("View at rotation {:5.3f}".format(rot))
        view.debugPrint(True, 0, 0)

    # adjust assumed rotation rot to rot'
    rotp = imap.getRotFit(view, Loc(0.0, 0.0, None, rot, float(step)/2))
    if DEBUG >= 3:
        print("Observed rotation: {:5.3f}".format(rotp))
    
    # remap position of observed objects to coordinate system of map
    view = view.getShiftedCopy(Loc(0.0, 0.0, None, rotp))
    
    # update map
    imap.update(view)

    # rotate by step degrees, adjust for rotp
    rotate(step + (rot-rotp))
    rot += step

    if DEBUG >= 2:
        print("Updated map")
        imap.debugPrint(True, 0, 0)
        print("Robot at (0.0, 0.0) with rotation {}\n".format(rotp))

if DEBUG:
    print("Stage I completed, resulting map:")
    imap.debugPrint(True, 0, 0)
    print("")



'''//------------------------------------------------------------------------//'
''// main code, part II: remapping to final map with more features, yay ----//''
'//------------------------------------------------------------------------//'''
print("+------------------------------------------------------------------------------+")
print("| REMAPPING STARTING                                                           |")
print("+------------------------------------------------------------------------------+\n")

# check which bottles are observed, create initial remap vector
yellow = imap.getBottle("yellow")
orange = imap.getBottle("orange")
blue = imap.getBottle("blue")
purple = imap.getBottle("purple")
x = y = False
rots = []
if yellow and purple:
    x = yellow.getDist(purple)
    rots.append((purple.getAngle(yellow))-pi/2)
if yellow and orange:
    y = yellow.getDist(orange)
    rots.append(orange.getAngle(yellow))
if orange and blue:
    rots.append((blue.getAngle(orange))-pi/2)
    if x:
        x = float(x+orange.getDist(blue)) / 2
    else:
        x = orange.getDist(blue)
if purple and blue:
    rots.append(blue.getAngle(purple))
    if y:
        y = float(y+purple.getDist(blue)) / 2
    else:
        y = purple.getDist(blue)

if not x or not y:
    print("Shit. Not enough bottles detected for proper remapping. But, let's continue anyway.")

rots = [-rot%(2*pi) for rot in rots]
rot = median(rots)
if rot != []:
    imap = imap.getShiftedCopy(Loc(0.0, 0.0, None, rot))

print("Remapping completed, resulting map:")
imap.debugPrint(True, 0, 0)

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
print("")



'''//------------------------------------------------------------------------//'
''// main code, part III: driving around, picking up bitches and boxes -----//''
'//------------------------------------------------------------------------//'''
print("+------------------------------------------------------------------------------+")
print("| STAGE II STARTING                                                            |")
print("+------------------------------------------------------------------------------+\n")
print("TODO: stage II\n")
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
print("+------------------------------------------------------------------------------+")
print("| CELEBRATING STARTING                                                         |")
print("+------------------------------------------------------------------------------+\n")
print("Yay! Beer?") 