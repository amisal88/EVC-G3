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

# get environment
# place objects on map
# rotate by rot degrees

rot = 30
for i in range(rot, 360, rot):
    # get environment
    # adjust assumed rotation rot to rot'
    # update map
    # rotate by rot degrees

# remap items on map to a new rectangular map using the given corner bottle order

# make a list of cone statuses

# make an empty list of boxes held by the robot

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

# while there are cones with undelivered boxes next to it
    # if gripper available
        # move to nearest delivery address OR nearest cone with undelivered boxes
    # else
        # move to nearest delivery address
    # if at delivery address
        # drop box
    # if at cone with undelivered boxes
        # pickup box        
        
# done
