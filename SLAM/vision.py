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

class Object_detector:
    def __init__(self, video_source, video_file):
	
		self.__blueLower = (78, 157, 73)
		self.__blueUpper = (106, 255, 255)

		self.__purpleLower = (142, 95, 74)
		self.__purpleUpper = (169, 255, 255)

		self.__orangeLower = (0, 93, 152)
		self.__orangeUpper = (39, 255, 255)

		self.__yellowLower = (26, 82, 112)
		self.__yellowUpper = (31, 255, 255)
		
		self.__color_codes_upper = [self.__blueUpper, self.__purpleUpper, self.__orangeUpper, self.__yellowUpper]
		self.__color_codes_lower = [self.__blueLower, self.__purpleLower, self.__orangeLower, self.__yellowLower]
		self.__color_names = ['Blue' , 'Purple', 'Orange', 'Yellow']
		
		self.__video_source = video_source
		
		if video_source == 0:
			self.__vs = VideoStream(usePiCamera=True).start()
		# otherwise, grab a reference to the video file
		else:
			self.__vs = cv2.VideoCapture(video_file)
			
		time.sleep(2.0)
		
	
    def process(self):

		# grab the current frame
		object_properties = {'name' : 'bottle', 'probability' : 0.0, 'position' : (0,0), 'stdev_p' : 0, 'rotation' : 0.0 , 'stdev_r': 0.0}
		objects_list = []
		frame = self.__vs.read()

		# handle the frame from VideoCapture or VideoStream
		frame = frame[1] if  (self.__video_source == 1) else frame

		# if we are viewing a video and we did not grab a frame,
		# then we have reached the end of the video
##		if frame is None:
##		    break

		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		
		for i in range(4):
				mask = cv2.inRange(hsv, self.__color_codes_lower[i], self.__color_codes_upper[i])
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
							cv2.putText(frame, self.__color_names[i], (cX - 20, cY - 20),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
							
							object_properties['name'] = 'BOTTLE'+self.__color_names[i];
							object_properties['position'] = (cX, cY)
							objects_list.append(object_properties)
							

							
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
									object_properties['name'] = 'CONE';
									object_properties['position'] = (cX, cY)
									objects_list.append(object_properties)
								else:
									cv2.putText(frame, "bottle_orange", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
									object_properties['name'] = BOTTLE+self.__color_names[i];
									object_properties['position'] = (cX, cY)
									objects_list.append(object_properties)
									
								

		# show the frame to our screen
		cv2.imshow("Frame", frame)


	# if we are not using a video file, stop the camera video stream
	#if self.__video_source == 0:
		#self.__vs.stop()

	# otherwise, release the camera
	#else:
		#self.__vs.release()

	# close all windows
	#cv2.destroyAllWindows()
	
		return objects_list
