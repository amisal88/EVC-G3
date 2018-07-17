from time import sleep
from numpy import inf, random
from math import sqrt, sin, cos, atan, pi
from copy import deepcopy
from statistics import median
#from vision import *
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
VERBOSE = 1     # debug print level: 0 = off, 1 = basic, 2 = medium, 3 = all
DEBUG = True    # enable/disable test environment
ADVANCED = True # test environment produces incorrect input/output
XSCALE = 16     # a terminal character width represents XSCALE centimeters
YSCALE = 34     # a terminal character height represents YSCALE centimeters



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

    # return copy of inverse
    def inverse(self):
        return Loc(-self.x, -self.y, self.pdev, -self.rot, self.rdev)

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

    # remap loc over distance (vec.x, vec.y), followed by rotation vec.rot
    # over (0.0, 0.0)
    def remap(self, vec):
        self.x += vec.x
        self.y += vec.y
        if vec.rot:
            newx = self.x * cos(vec.rot) + self.y * sin(vec.rot)
            self.y = self.x * -sin(vec.rot) + self.y * cos(vec.rot)
            self.x = newx
            if self.rot != None:
                self.rot += vec.rot

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
        elif self.rot != None:
            wrot = self.rot
        else:
            wrot = loc.rot
        if self.rdev != None and loc.rdev != None:
            wrdev = float(pow(self.rdev,w)*pow(loc.rdev,iw))/sqrt(2)
        elif self.rdev != None:
            wrdev = self.rdev
        else:
            wrdev = loc.rdev
        return Loc(wx, wy, wpdev, wrot, wrdev)

    def matchError(self, loc):
        dst = self.getDist(loc)
        if self.pdev and loc.pdev:
            magic = 1 + self.pdev + loc.pdev # TODO: tweak this formula
        else:
            magic = 10 # TODO: tweak this value
        dsterr = float(dst) / magic
        if self.rot and loc.rot:
            angle = min(abs(self.rot-loc.rot), abs(loc.rot-self.rot)) % 2*pi
            if self.rdev and loc.rdev:
                magic = 1 + self.rdev + loc.rdev # TODO: tweak this formula
            else:
                magic = 1 # TODO: tweak this value
            roterr = float(angle) / magic
        else:
            roterr = 0
        return dsterr + roterr


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
        self.pos.remap(vec)

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
        weight_pdev = 1 - float(self.pos.pdev+1) / (self.pos.pdev+obj.pos.pdev+2)
        weight_total = 0.0*weight_prob + 0.7*weight_views + 0.3*weight_pdev # TODO: tweak these values
        if VERBOSE >= 3:
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

    # getters/setters
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
# the current robot position on the map is always (0,0) with viewing direction
# of positive y axis. The robot does not move, the map moves.
class Map(object):
    # class variables
    # objs = []
    # marks = []

    # constructor
    def __init__(self):
        self.objs = []
        self.marks = []

    # getters/setters
    def getBottle(self, color):
        bottles = [obj for obj in self.objs if isinstance(obj, Bottle) and obj.color == color]
        if bottles == []:
            return False
        maxprob = max([bottle.prob for bottle in bottles])
        for bottle in bottles:
            if bottle.prob == maxprob:
                return bottle

    def getCones(self):
        return [obj for obj in self.objs if isinstance(obj, Cone)]

    def getLastMark(self):
        return self.marks[-1]

    def setSize(self, x, y):
        self.width = x
        self.length = y

    # add new objects to the map
    def add(self, obj):
        self.objs.append(obj)

    # add robot location: call this function every time getSurroundings() is callid
    def addMark(self):
        self.marks.append(Loc(0.0, 0.0, 0, 0, 0))

    # shift all map elements over distance (vec.x, vec.y) then rotate by
    # vec.rot radians over point (0.0, 0.0)
    def shiftMap(self, vec):
        for obj in self.objs:
            obj.remap(vec)
        for mark in self.marks:
            mark.remap(vec)

    # duplicate map, shift all elements over distance (vec.x, vec.y), then
    # rotate by vec.rot radians over point (0.0, 0.0)
    def getShiftedCopy(self, vec):
        copy = deepcopy(self)
        copy.shiftMap(vec)
        return copy

    # find match between objects on map and new object. If no match found,
    # return false
    def findMatch(self, obj):
        minerr = 2 # TODO: tweak this value
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
    def getRotFit(self, mapb, vec=None):
        if vec == None:
            vec = Loc(0.0, 0.0, inf, 0.0, 2*pi)
        copy = mapb.getShiftedCopy(vec)
        rdif = []
        for obj in copy.objs:
            match = self.findMatch(obj)
            if match:
                rota = obj.pos.getAngle()
                rotb = match.pos.getAngle()
                drot = (rotb-rota+pi) % (2*pi) - pi
                if VERBOSE >= 3:
                    print("rota: {:5.3}\trotb: {:5.3}\tdrot: {:5.3}".format(rota, rotb, drot))
                if abs(drot) < vec.rdev:
                    rdif.append(drot)
        if len(rdif):
            vec.rot += median(rdif)
        return vec

    # returns best location shift fit for map B on map A. Arguments: self = map
    # A; # mapb = map B, (vec.x, vec.y) = location shift starting point,
    # vec.pdev = maximum shift deviation, vec.rot = rotation shift
    def getPosFit(self, mapb, vec=None):
        if vec == None:
            vec = Loc(0.0, 0.0, inf, 0.0, 2*pi)
        copy = mapb.getShiftedCopy(vec)
        weight = 0.0
        sumx = sumy = 0.0
        for obj in copy.objs:
            match = self.findMatch(obj)
            if match:
                dx = obj.pos.x - match.pos.x
                dy = obj.pos.y - match.pos.y
                magic = (1 + obj.pos.pdev + match.pos.pdev) # TODO: tweak this formula
                if VERBOSE>3:
                    print(dx, dy, magic)
                sumx += dx/magic
                sumy += dy/magic
                weight += 1/magic
        if weight == 0:
            return vec
        dx = sumx/weight
        dy = sumy/weight
        if vec.pdev:
            dx = min(dx, vec.pdev)
            dx = max(dx, -vec.pdev)
            dy = min(dy, +vec.pdev)
            dy = max(dy, -vec.pdev)
        vec.x += dx
        vec.y += dy
        return vec

    # returns best rotation + location shift fit for map B on map A. Arguments:
    # self = map A; # mapb = map B, (vec.x, vec.y) = location shift starting
    # point, vec.pdev = maximum shift deviation, vec.rot = rotation starting
    # point, vec.rdev = maximum rotation deviation
    def getRotPosFit(self, mapb, vec):
        veca = deepcopy(vec)
        if VERBOSE >= 3:
            print("getRotPosFit: {}".format(vec))
        delta = inf
        err = inf
        while (delta >= 10): # TODO: tweak this value
            vecb = self.getRotFit(mapb, veca)
            vecc = self.getPosFit(mapb, vecb)
            errp = vec.matchError(vecc)
            delta = abs(err - errp)
            veca = vecc
            err = errp
            if VERBOSE >= 3:
                print("err: {:5.2f}\tdelta: {:5.2f}\tvec:{}".format(err, delta, vec))
        return vec

    # find the four corner bottles, calculate reshaping of map using fact that
    # map should be rectangular, adjust objects on map using weight w
    def squareMap(self, w):
        if VERBOSE >= 2:
            print("Squaring corners:")
        yellow = imap.getBottle("yellow")
        orange = imap.getBottle("orange")
        blue = imap.getBottle("blue")
        purple = imap.getBottle("purple")
        if not (yellow and orange or blue and purple) or not (orange and blue or purple and yellow):
            if VERBOSE >= 2:
                print("Not enough bottles detected for proper squaring")
        print("Not yet implemented!") # TODO: implement

    # Update map with objects from given view
    def update(self, view):
        for obj in view.objs:
            match = self.findMatch(obj)
            if match:
                match.update(obj)
            else:
                self.add(obj)

    # debug print: print all objects
    def debugPrint(self):
        for obj in self.objs:
            print("{}\t{}".format(self.objs.index(obj), obj))
        if self.marks != []:
            for mark in self.marks:
                print("{}\t{}".format(self.marks.index(mark), mark))
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
                if round(float(-xmin)/XSCALE) == x and round(float(-ymin)/YSCALE) == y:
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
if DEBUG:
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
                if obj.pos.getDist(Loc(x, y)) < 200:
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
    debugmap.shiftMap(Loc(-robotx, -roboty, None, -robotrot))
    robot = Obj(1.0, Loc(0.0, 0.0, 0.0, 0.0, 0.0))
    print("Debug map created:")
    debugmap.debugPrint()
    print("Robot placed at position ({}, {}) with rotation {:5.3f}\n".format(robotx, roboty, robotrot))


# class Vision, used for communication with vision part ---------------------- #
class Vision:
    # class variables
    # vision

    # constructor
    def __init__(self):
        if not DEBUG:
            self.vision = Object_detector(0)

    # getSurroundings
    def getSurroundings(self):
        if VERBOSE >= 3:
            print("getSurroundings()")
        if DEBUG:
            objects_list = []
            for obj in debugmap.objs:
                dist = obj.getDist(robot)
                angle = (obj.getAngle(robot) + pi) % (2*pi) - pi
                if angle > (-pi/3) and angle < (pi/3) and not isinstance(obj, Box):
                    if VERBOSE >= 3:
                        print("\tObject: {}".format(obj))
                        print("\tObject in sight? Dist: {:5.1f}, angle: {:5.2f}, prob: {:4.2f}.".format(dist, angle, 400.0/(dist+400)))
                    if not ADVANCED or 400.0/(dist+400) >= random.rand():
                        if ADVANCED:
                            prob = 0.8
                            magic = 0.8 - 480.0/(dist+600)
                            dist += (magic*random.rand() - magic/2) * dist
                            pdev = magic*dist/2+10
                            magic = 50.0/(dist+25)
                            angle += (magic*random.rand() - magic/2)
                        else:
                            prob = 1.0
                            pdev = 0.0
                        if VERBOSE >= 3:
                            print("\t\tYes, observed at dist {:5.1f} and angle {:5.2f}".format(dist, angle))
                        posx = dist * sin(angle)
                        posy = dist * cos(angle)
                        if isinstance(obj, Bottle):
                            objects_list.append({'name': 'bottle', 'probability': prob, 'position': (posx,posy), 'stdev_p': pdev, 'color': obj.color})
                        elif isinstance(obj, Cone):
                            objects_list.append({'name': 'cone', 'probability': prob, 'position': (posx,posy), 'stdev_p': pdev})
                    else:
                        if VERBOSE >= 3:
                            print("\t\tNope, too cloudy")
            if VERBOSE >= 3:
                print("")
            return objects_list
        else:
            return self.vision.objects_process();

    def getView(self):
        if DEBUG >= 3:
            print("getView()")
        view = Map()
        for i in self.getSurroundings():
            view.add(parse(i))
        return view


# class Arduino, used to link control commands to the Arduino communication -- #
class Arduino:
    # class variables
    # port

    # constructor
    def __init__(self):
        if not DEBUG:
            self.port = serial.Serial('/dev/ttyUSB0', 9600)
            sleep(2)

    # write command
    def writeM(self, m):
        self.port.write(m)

    # wait for the OK message
    def readOk(self):
        while True:
            m = self.port.readline()
            if "OK" in m:
                break

    # read vector message
    def readVec(self):
        m = self.port.readline()
        m = m.split("(")[1]
        m = m.split(")")[0]
        v = m.split(" , ")
        return Loc(float(v[0]), float(v[1]), float(v[2]), float(v[3]), float(v[4]))

    # measure power
    def readPower(self):
        if DEBUG:
            print("TODO: implement measure") # TODO: implement
            return None
        else:
            print("Power stats:")
            self.writeM("measure")
            for i in range(0,8):
                print("\t{}".format(self.port.readline().split("\n")[0]))

    # grab(n)
    def boxGrab(self, n):
        if DEBUG:
            print("TODO: implement grab(n)") # TODO: implement
            return None
        else:
            print("TODO: implement grab(n)") # TODO: implement
            return None

    # deliver(n)
    def boxDrop(self, n):
        if DEBUG:
            print("TODO: implement deliver(n)") # TODO: implement
            return None
        else:
            print("TODO: implement deliver(n)") # TODO: implement
            return None

    # XYmove(x, y) and XYpose(x, y, a)
    def move(self, vec):
        if DEBUG:
            if VERBOSE >= 3:
                print("move({})".format(dist))
            if not vec.rot:
                vec.rot = vec.getAngle() # TODO: replace this formula with correct formula
            if ADVANCED:
                vec.x += 40*random.rand() - 20
                vec.y += 40*random.rand() - 20
                vec.rot += pi/4.5*random.rand() - pi/9
                if VERBOSE >= 3:
                    print("Actual move: {}".format(vecp))
            debugmap.shiftMap(vec.inverse())
            if ADVANCED:
                vec.x += 20*random.rand() - 10
                vec.y += 20*random.rand() - 10
                vec.rot += pi/9*random.rand() - pi/18
                if VERBOSE >= 3:
                    print("Returned move: {}".format(vecp))
            return vec
        else:
            if vec.rot:
                m = "XYpos(" + str(vec.x) + ", " + str(vec.y) + ", " + str(vec.rot) + ")"
            else:
                m = "XYmove(" + str(vec.x) + ", " + str(vec.y) + ")"
            self.writeM(m)
            return self.readVec()

    # rotate(a)
    def rotate(self, angle):
        if VERBOSE >= 3:
            print('Rotate by {:5.3f} radians\n'.format(angle))
        if DEBUG:
            global debugmap
            vec = Loc(0.0, 0.0, None, angle)
            if ADVANCED:
                vec.rot += (pi/4.5*random.rand() - pi/9)
                if VERBOSE >= 3:
                    print("Actual rotation: {:5.3} radians".format(vec.rot))
            debugmap.shiftMap(vec.inverse())
            if ADVANCED:
                vec.rot += (pi/9*random.rand() - pi/18)
                if VERBOSE >= 3:
                    print("Returned rotation: {:5.3} radians".format(vec.rot))
            return vec
        else:
            self.writeM("rotate(" + str(angle/2) + ")")
            return self.readVec()

    # slamrot()
    def slamrot(self):
        if VERBOSE >= 3:
            print('Slamrot')
        if DEBUG:
            global debugmap
            vec = Loc(0.0, 0.0, None, pi/6)
            if ADVANCED:
                vec.rot += (pi/18*random.rand() - pi/36)
                if VERBOSE >= 3:
                    print("Actual rotation: {:5.3} radians".format(vec.rot))
            debugmap.shiftMap(vec.inverse())
            vec = Loc(0.0, 0.0, None, pi/6)
            return vec
        else:
            self.writeM("slamrot")
            return self.readVec()

    # backoff(y)
    def backoff(self, dist):
        if DEBUG:
            print("TODO: implement backoff(dist)") # TODO: implement
            return None
        else:
            print("TODO: implement backoff(dist)") # TODO: implement
            return None



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
    dist = sqrt(x**2 + y**2)
    pdev = (dist/2000+0.1)*dist # TODO: fix this still unwanted situation
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

# initialize vision communication
sauron = Vision()

# initialize Arduino communication
ardy = Arduino()

# create empty initial map
imap = Map()

# get environment
environment = sauron.getSurroundings()
imap.addMark()

ardy.readPower() # TMP FOR PRESENTATION: PRINT POWER STATS

# place objects on map
for i in environment:
    imap.add(parse(i))

if VERBOSE >= 2:
    print("Initial view, starting map:")
    imap.debugPrint()
    print("Robot at (0.0, 0.0) with rotation 0.0\n")

# rotation settings
steps = 12
step = 2 * pi / steps
rot = 0
while (rot < 2*pi - step/2):
    # rotate by step degrees
    rvec = ardy.slamrot()

    # get view
    view = sauron.getView()

    # adjust assumed rotation step
    pvec = imap.getRotFit(view, Loc(0.0, 0.0, None, pi/6))
    vec = rvec.wavg(pvec, 0.8)
    rot += vec.rot
    
    # remap position of observed objects to coordinate system of map
    imap.shiftMap(vec.inverse())
    imap.addMark()

    ardy.readPower() # TMP FOR PRESENTATION: PRINT POWER STATS

    # update map
    imap.update(view)
    if VERBOSE >= 2:
        print("Updated map")
        imap.debugPrint()

if VERBOSE:
    print("Stage I completed, resulting map:")
    imap.debugPrint()
    print("")



'''//------------------------------------------------------------------------//'
''// main code, part II: remapping to final map with more features, yay ----//''
'//------------------------------------------------------------------------//'''
print("+------------------------------------------------------------------------------+")
print("| REMAPPING STARTING                                                           |")
print("+------------------------------------------------------------------------------+\n")

# square map a bit
#imap.squareMap(0.5) # TODO: tweak this value
#print("Map remapped, resulting map:")
#imap.debugPrint()
if DEBUG and VERBOSE >= 2:
    print(debugmap.getRotFit(imap))
    print(debugmap.getPosFit(imap))

# initialize gripper
gripper = [None, None, None]

'''
At this point we have a nice map containing the following:
- TODO: exactly four corner bottles
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

# repeat grabbing and dropping of boxes until done
done = False
while not done:
    if VERBOSE >= 2:
        print("Let's drive!")

    # make list of unvisited cones and box destination cone
    newcones = [cone for cone in imap.getCones() if cone.toVisit()]
    destcones = [box.dest for box in gripper if box != None and isinstance(box.dest, Cone)]
    cones = newcones + destcones

    # select nearest cone
    dists = [cone.getDist() for cone in cones]
    cone = cones[dists.index(min(dists))]
    if VERBOSE >= 2:
        print("Destination: {}".format(cone))

    # drive to nearest relevant cone
    dist = cone.getDist()
    if cone.pos.y < 20 or dist < 50: # TODO: tweak these values
        if VERBOSE >= 2:
            print("ROTATE")
        angle = cone.getAngle()
        vecr = ardy.rotate(angle)
        view = sauron.getView()
        vecp = imap.getRotFit(view, vecr)
        vec = vecr.wavg(vecp, 0.8) # TODO: tweak this value
        imap.shiftMap(vec.inverse())
        imap.addMark()
        if VERBOSE >= 3:
            imap.debugPrint()
    dist = cone.getDist()
    if dist > 50: # TODO: tweak this value
        if VERBOSE >= 2:
            print("ROLL")
        dist -= 40 # TODO: tweak this value
        angle = cone.getAngle()
        vecm = ardy.move(Loc(dist*sin(angle), dist*cos(angle)))
        view = sauron.getView()
        vecp = imap.getRotPosFit(view, vecm)
        vec = vecm.wavg(vecp, 0.8) # TODO: tweak this value
        imap.shiftMap(vec.inverse())
        imap.addMark()
        if VERBOSE >= 3:
            imap.debugPrint()

    # if at unidentified cone:
        # identify cone

    # if at delivery address
        # drop box
        # set box to delivered

    # if at cone with undelivered boxes
        # pickup box
        # read box data
        # if last box at cone
            # cone.boxesDone = True

    # if list of unvisited cones is empty and list of undelivered boxes is empty
        # done = True

    ardy.readPower() # TMP FOR PRESENTATION: PRINT POWER STATS

    if VERBOSE >= -2:
        print("Updated map")
        imap.debugPrint()
    exit()



'''//------------------------------------------------------------------------//'
''// main code, part IV: done ----------------------------------------------//''
'//------------------------------------------------------------------------//'''
print("+------------------------------------------------------------------------------+")
print("| CELEBRATING STARTING                                                         |")
print("+------------------------------------------------------------------------------+\n")
print("Yay! Beer?")
