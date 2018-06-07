# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

def convex_hull_pointing_up(ch):
    points_above_center, points_below_center = [], []
    
    x, y, w, h = cv2.boundingRect(ch)
    aspect_ratio = w / h
    if aspect_ratio < 0.8:
        vertical_center = y + h / 2

        for point in ch:
            if point[0][1] < vertical_center:
                points_above_center.append(point)
            elif point[0][1] >= vertical_center:
                points_below_center.append(point)
				
        left_x = points_below_center[0][0][0]
        right_x = points_below_center[0][0][0]
        for point in points_below_center:
            if point[0][0] < left_x:
                left_x = point[0][0]
            if point[0][0] > right_x:
                right_x = point[0][0]

        for point in points_above_center:
            if (point[0][0] < left_x) or (point[0][0] > right_x):
                return False
    else:
        return False
        
    return True

class Robot_vision:
    def __init__(self):
        object_properties = {'bottle' , 'name' : 'bottle', 'probability' : 0.0, 'position' : (0,0), 'stdev_p' : 0, 'rotation' : 0.0 , 'stdev_r': 0.0}
        objects_list = []
    

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points


blueLower = (78, 157, 73)
blueUpper = (106, 255, 255)

purpleLower = (142, 95, 74)
purpleUpper = (169, 255, 255)

orangeLower = (0, 93, 152)
orangeUpper = (39, 255, 255)

yellowLower = (26, 82, 112)
yellowUpper = (31, 255, 255)

color_codes_upper = [blueUpper, purpleUpper, orangeUpper, yellowUpper]
color_codes_lower = [blueLower, purpleLower, orangeLower, yellowLower]
color_names = ['Blue' , 'Purple', 'Orange', 'Yellow']

pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(usePiCamera=True).start()

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)
# keep looping
while True:
	# grab the current frame
	frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	
	for i in range(4):
            mask = cv2.inRange(hsv, color_codes_lower[i], color_codes_upper[i])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            center = None

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    if i != 2: # not orange
                        c = max(cnts, key=cv2.contourArea)
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
        ##		((x, y), radius) = cv2.minEnclosingCircle(c)
        ##		M = cv2.moments(c)
        ##		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                        cv2.drawContours(frame,[c],0,(0,255,0),3)
                        cv2.putText(frame, color_names[i], (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        # only proceed if the radius meets a minimum size
        ##		if radius > 10:
        ##			# draw the circle and centroid on the frame,
        ##			# then update the list of tracked points
        ##			cv2.circle(frame, (int(x), int(y)), int(radius),
        ##				(0, 255, 255), 2)
        ##			cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        
                    else:
                        img_edges = cv2.Canny(mask.copy(), 80, 160)
                        _, contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        
                        approx_contours = []
                        for c in contours:
                            approx = cv2.approxPolyDP(c, 10, closed = True)
                            approx_contours.append(approx)
                        
                        all_convex_hulls = []
                        for ac in approx_contours:
                            all_convex_hulls.append(cv2.convexHull(ac))
                            
                        convex_hulls_3to10 = []
                        for ch in all_convex_hulls:
                            if 3 <= len(ch) <= 10:
                                convex_hulls_3to10.append(cv2.convexHull(ch))
                                
                        
                        for c in convex_hulls_3to10:
                            M = cv2.moments(c)
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            cv2.drawContours(frame,[c],0,(0,255,0),3)
                            if convex_hull_pointing_up(c):
                                cv2.putText(frame, "cone", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            else:
                                cv2.putText(frame, "bottle_orange", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                                
                                
##                        c = sorted(cnts, key=cv2.contourArea)
##                        if len(c) > 4:
##                            c_up = 4
##                        else:
##                            c_up = len(c)
##                        
##                        for j in range(c_up):
##                            c_orange = c[j]
##                            shape = sd.detect(c_orange)
##                            M = cv2.moments(c_orange)
##                            cX = int(M["m10"] / M["m00"])
##                            cY = int(M["m01"] / M["m00"])
##            ##		((x, y), radius) = cv2.minEnclosingCircle(c)
##            ##		M = cv2.moments(c)
##            ##		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
##                            cv2.drawContours(frame,[c_orange],0,(0,255,0),3)
##                            cv2.putText(frame, shape, (cX - 20, cY - 20),
##                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    

            # update the points queue
            pts.appendleft(center)

            # loop over the set of tracked points
    ##	for i in range(1, len(pts)):
    ##		# if either of the tracked points are None, ignore
    ##		# them
    ##		if pts[i - 1] is None or pts[i] is None:
    ##			continue
    ##
    ##		# otherwise, compute the thickness of the line and
    ##		# draw the connecting lines
    ##		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
    ##		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()