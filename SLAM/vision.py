# import the necessary packages
from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import RPi.GPIO as gp
from pyzbar import pyzbar
import math
import pickle

def intersect(l1, l2):
	delta = np.array([l1[1] - l1[0], l2[1] - l2[0]]).astype(np.float32)
	
	delta = 1 / delta
	delta[:, 0] *= -1
	
	b = np.matmul(delta, np.array([l1[0], l2[0]]).transpose())
	b = np.diagonal(b).astype(np.float32)
		
	res = cv2.solve(delta, b)
	return res[0], tuple(res[1].astype(np.int32).reshape((2)))

def rectify(image, corners, out_size):
	rect = np.zeros((4, 2), dtype = "float32")
	rect[0] = corners[0]
	rect[1] = corners[1]
	rect[2] = corners[2]
	rect[3] = corners[3]

	dst = np.array([
		[0, 0],
		[out_size[1] - 1, 0],
		[out_size[1] - 1, out_size[0] - 1],
		[0, out_size[0] - 1]], dtype = "float32")

	M = cv2.getPerspectiveTransform(rect, dst)
	rectified = cv2.warpPerspective(image, M, out_size)
	return rectified

def qr_code_outer_corners(image):
	outer_corners_found = False
	outer_corners = []
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	_, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
	
	_, contours, hierarchy = \
			cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
	cnts = []
	centers = []
		
	hierarchy = hierarchy.reshape((-1, 4))
	for i in range(hierarchy.shape[0]):
		i_next, i_prev, i_child, i_par = hierarchy[i]
		if all(v == -1 for v in hierarchy[i][:3]):
			if all(v == -1 for v in hierarchy[i_par][:2]):
				ids = [i, i_par, hierarchy[i_par][3]]
				corner_cnts = []
				for id_ in ids:
					cnt = contours[id_]
					apprx = \
						cv2.approxPolyDP(cnt, cv2.arcLength(cnt, True) * 0.06, True)
					if len(apprx) == 4:
						corner_cnts.append(apprx.reshape((4, -1)))
				if len(corner_cnts) == 3:
					cnts.append(corner_cnts)
					all_pts = np.array(corner_cnts).reshape(-1, 2)
					
					centers.append(np.mean(all_pts, 0))

	
	if len(centers) == 3:		
		distances_between_pts = np.linalg.norm(np.roll(centers, 1, 0) - centers, axis=1)
		max_dist_id = np.argmax(distances_between_pts)
		
		index_diag_pt_1 = max_dist_id
		index_diag_pt_2 = (max_dist_id - 1) % len(centers)
		index_corner_pt = (len(centers) - 1)*len(centers) // 2 - index_diag_pt_1 - index_diag_pt_2
		
		middle_pt = 0.5 * (centers[index_diag_pt_1] + centers[index_diag_pt_2])
		
		i_ul_pt = np.argmax(np.linalg.norm(cnts[index_corner_pt][-1] - middle_pt, axis=1))
		ul_pt = cnts[index_corner_pt][-1][i_ul_pt]
				
		for i in [index_diag_pt_1, index_diag_pt_2]:
			corner_cnts = cnts[i]
			outer_cnt = corner_cnts[-1]
			
			distances_to_mp = np.linalg.norm(outer_cnt - middle_pt, axis=1)
			max_dist_id = np.argmax(distances_to_mp)	  
		
			vec_from_mid_to_diag = outer_cnt[max_dist_id] - middle_pt
			vec_from_mid_to_corner = ul_pt - middle_pt
			cross_prod = np.cross(vec_from_mid_to_corner, vec_from_mid_to_diag)
		
			diff_idx = 0
		
			if cross_prod > 0:
				ur_pt = outer_cnt[max_dist_id]
				ur_pt_2 = outer_cnt[(max_dist_id + 1) % len(outer_cnt)]
			else:
				bl_pt = outer_cnt[max_dist_id]
				bl_pt_2 = outer_cnt[(max_dist_id - 1) % len(outer_cnt)]
					
		ret, br_pt = intersect((bl_pt, bl_pt_2), (ur_pt, ur_pt_2))
		
		if ret == True:
			outer_corners_found = True
			outer_corners = [ul_pt, ur_pt, br_pt, bl_pt]
	
	return outer_corners_found, outer_corners

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
	def __init__(self, debug_on):
	
		self.__focalLength = 303.4005
		file = open('Left_Stereo_Map_file.txt', 'r')
		self.__Left_Stereo_Map = pickle.load(file)
		file.close()
		file = open('Right_Stereo_Map_file.txt', 'r')
		self.__Right_Stereo_Map = pickle.load(file)
		file.close()
		self.__window_size = 3
		self.__min_disp = 2
		self.__num_disp = 130-self.__min_disp
		self.__stereo = cv2.StereoSGBM_create(minDisparity = self.__min_disp,
			numDisparities = self.__num_disp,
			blockSize = self.__window_size,
			uniquenessRatio = 10,
			speckleWindowSize = 100,
			speckleRange = 32,
			disp12MaxDiff = 5,
			P1 = 8*3*self.__window_size**2,
			P2 = 32*3*self.__window_size**2)

		# Used for the filtered image
		self.__stereoR=cv2.ximgproc.createRightMatcher(self.__stereo) # Create another stereo for right this time

		# WLS FILTER Parameters
		self.__lmbda = 80000
		self.__sigma = 1.8
		self.__visual_multiplier = 1.0
		 
		self.__wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.__stereo)
		self.__wls_filter.setLambda(self.__lmbda)
		self.__wls_filter.setSigmaColor(self.__sigma)

		self.__debug_on = debug_on
		self.__blueLower = (78, 157, 73)
		self.__blueUpper = (106, 255, 255)

		self.__purpleLower = (142, 95, 74)
		self.__purpleUpper = (169, 255, 255)

		self.__orangeLower = (32, 153, 40)
		self.__orangeUpper = (214, 244, 255)

		self.__yellowLower = (26, 82, 112)
		self.__yellowUpper = (31, 255, 255)
		
		self.__color_codes_upper = [self.__blueUpper, self.__purpleUpper, self.__orangeUpper, self.__yellowUpper]
		self.__color_codes_lower = [self.__blueLower, self.__purpleLower, self.__orangeLower, self.__yellowLower]
		self.__color_names = ['BLUE' , 'PURPLE', 'ORANGE', 'YELLOW']
		
		self.__DIM=(640, 480)
		self.__KL=np.array([[310.89487794241705, 0.0, 310.4701490632405], [0.0, 311.6045360428195, 217.332401108332], [0.0, 0.0, 1.0]])
		self.__DL=np.array([[-0.02678943147680496], [0.07199621192297553], [-0.17537795442931486], [0.1224981141497315]])
		self.__KR=np.array([[311.9993342853497, 0.0, 288.39779549515345], [0.0, 312.9238338162564, 233.89810329394894], [0.0, 0.0, 1.0]])
		self.__DR=np.array([[-0.035125623121583065], [0.0721407010760906], [-0.1341791663672435], [0.07146034215266592]])
		
		# initialize the camera and grab a reference to the raw camera capture
		gp.setwarnings(False)
		gp.setmode(gp.BOARD)
		gp.setup(7, gp.OUT)
		gp.setup(11, gp.OUT)
		gp.setup(12, gp.OUT)
		gp.setup(15, gp.OUT)
		gp.setup(16, gp.OUT)
		gp.setup(21, gp.OUT)
		gp.setup(22, gp.OUT)

		gp.output(11, True)
		gp.output(12, True)
		gp.output(15, True)
		gp.output(16, True)
		gp.output(21, True)
		gp.output(22, True)

		gp.output(7, False)
		gp.output(11, False)
		gp.output(12, True)
		
		self.__camera = PiCamera()
		self.__camera.resolution = (640, 480)
		self.__rawCapture = PiRGBArray(self.__camera)
			
		time.sleep(2.0)
		
	
	def objects_process(self):

		# grab the current frame
		object_properties = {'name' : None, 'probability' : 0.0, 'position' : (0,0), 'distance' : 0.0 ,'stdev_p' : 0, 'rotation' : 0.0 , 'stdev_r': 0.0}
		objects_list = []
		self.__camera.capture(self.__rawCapture, format="bgr")
		frameR = self.__rawCapture.array
		map1R, map2R = cv2.fisheye.initUndistortRectifyMap(self.__KR, self.__DR, np.eye(3), self.__KR, self.__DIM, cv2.CV_16SC2)
		frame = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
		self.__rawCapture.truncate(0)
		# resize the frame, blur it, and convert it to the HSV color space
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		
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
							
							if (self.__debug_on == 1):
								cv2.drawContours(frame,[c],0,(0,255,0),3)
								cv2.putText(frame, self.__color_names[i], (cX - 20, cY - 20),
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
							
							object_properties['name'] = 'BOTTLE_'+self.__color_names[i];
							object_properties['position'] = (cX, cY)
							marker = cv2.minAreaRect(c)
							object_properties['distance'] = (26.0 * self.__focalLength) / marker[1][1]
							objects_list.append(object_properties.copy())
							# focal length calibration 
							KNOWN_DISTANCE = 60.0
							KNOWN_HEIGHT = 26.0
							marker = cv2.minAreaRect(c)
							focalLength = (marker[1][1] * KNOWN_DISTANCE) / KNOWN_HEIGHT
							print 'focalLength'
							print focalLength
							
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
								
								if convex_hull_pointing_up(c):
									if (self.__debug_on == 1):
										cv2.drawContours(frame,[c],0,(0,255,0),3)
										cv2.putText(frame, "cone", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
									object_properties['name'] = 'CONE';
									object_properties['position'] = (cX, cY)
									marker = cv2.minAreaRect(c)
									object_properties['distance'] = (17.5 * self.__focalLength) / marker[1][1]
									objects_list.append(object_properties.copy())
								else:
									if (self.__debug_on == 1):
										cv2.drawContours(frame,[c],0,(0,255,0),3)
										cv2.putText(frame, "bottle_orange", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
									object_properties['name'] = 'BOTTLE_'+self.__color_names[i];
									object_properties['position'] = (cX, cY)
									marker = cv2.minAreaRect(c)
									object_properties['distance'] = (26.0 * self.__focalLength) / marker[1][1]
									objects_list.append(object_properties.copy())
									
								

		# show the frame to our screen
		if (self.__debug_on == 1):
			cv2.imshow("Frame", frame)
			cv2.waitKey(100)
		return objects_list

	def qr_process(self):

		self.__camera.capture(self.__rawCapture, format="bgr")
		frameR = self.__rawCapture.array
		map1R, map2R = cv2.fisheye.initUndistortRectifyMap(self.__KR, self.__DR, np.eye(3), self.__KR, self.__DIM, cv2.CV_16SC2)
		image = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
		self.__rawCapture.truncate(0)
		result, corners = qr_code_outer_corners(image)
		qr_code_size = 100
		QR_code_properties = {'name' : None, 'angle' : None}
	
		if result:
			if all((0, 0) < tuple(c) < (image.shape[1], image.shape[0]) for c in corners):
				rectified = rectify(image, corners, (qr_code_size, qr_code_size))
				angle = int(math.atan((float)((corners[0][1]-corners[1][1]))/((float)(corners[1][0]-corners[0][0])))*180/math.pi)
				QR_code_properties['angle'] = angle
				
				if (self.__debug_on == 1):
					print angle
					cv2.circle(image, tuple(corners[0]), 15, (0, 255, 0), 2)
					cv2.circle(image, tuple(corners[1]), 15, (0, 0, 255), 2)
					cv2.circle(image, tuple(corners[2]), 15, (255, 0, 0), 2)
					cv2.circle(image, tuple(corners[3]), 15, (255, 255, 0), 2)
					image.flags.writeable = True
					image[0:qr_code_size, 0:qr_code_size] = rectified

				barcodes = pyzbar.decode(rectified)

				# loop over the detected barcodes
				for barcode in barcodes:
					barcodeData = barcode.data.decode("utf-8")
					barcodeType = barcode.type
					QR_code_properties['name'] = format(barcodeData).encode('utf-8') 
		if (self.__debug_on == 1):
			cv2.imshow('QR code', image)
			k = cv2.waitKey(100)		
		return QR_code_properties

	
	def stereo_objects_process(self):

		# grab the current frame
		object_properties = {'name' : None, 'probability' : 0.0, 'position' : (0,0), 'distance' : 0.0 ,'stdev_p' : 0, 'rotation' : 0.0 , 'stdev_r': 0.0}
		objects_list = []
		self.__camera.capture(self.__rawCapture, format="bgr")
		frameR = self.__rawCapture.array
		self.__rawCapture.truncate(0)
		gp.output(7, False)
		gp.output(11, True)
		gp.output(12, False)
		time.sleep(0.1)
		self.__camera.capture(self.__rawCapture, format="bgr")
		frameL = self.__rawCapture.array
		self.__rawCapture.truncate(0)
		gp.output(7, False)
		gp.output(11, False)
		gp.output(12, True)
		
		# Rectify the images on rotation and alignement
		Left_nice= cv2.remap(frameL,self.__Left_Stereo_Map[0],self.__Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  # Rectify the image using the kalibration parameters founds during the initialisation
		Right_nice= cv2.remap(frameR,self.__Right_Stereo_Map[0],self.__Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
		# Convert from color(BGR) to gray
		grayR= cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
		grayL= cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)
		# Compute the 2 images for the Depth_image
		disp= self.__stereo.compute(grayL,grayR)#.astype(np.float32)/ 16
		dispL= disp
		dispR= self.__stereoR.compute(grayR,grayL)
		dispL= np.int16(dispL)
		dispR= np.int16(dispR)

		# Using the WLS filter
		filteredImg= self.__wls_filter.filter(dispL,grayL,None,dispR)
		filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
		filteredImg = np.uint8(filteredImg)
		#cv2.imshow('Disparity Map', filteredImg)
		disp= ((disp.astype(np.float32)/ 16)-self.__min_disp)/self.__num_disp # Calculation allowing us to have 0 for the most distant object able to detect
		
		#map1R, map2R = cv2.fisheye.initUndistortRectifyMap(self.__KR, self.__DR, np.eye(3), self.__KR, self.__DIM, cv2.CV_16SC2)
		#frame = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
		frame = Right_nice
		# resize the frame, blur it, and convert it to the HSV color space
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		
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
							
							if (self.__debug_on == 1):
								cv2.drawContours(frame,[c],0,(0,255,0),3)
								cv2.putText(frame, self.__color_names[i], (cX - 20, cY - 20),
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
							
							object_properties['name'] = 'BOTTLE_'+self.__color_names[i];
							object_properties['position'] = (cX, cY)
							marker = cv2.minAreaRect(c)
							object_properties['distance'] = (32.0 * 0.51)/disp[cY, cX]
							objects_list.append(object_properties.copy())
							# focal length calibration 
							KNOWN_DISTANCE = 60.0
							KNOWN_HEIGHT = 26.0
							marker = cv2.minAreaRect(c)
							focalLength = (marker[1][1] * KNOWN_DISTANCE) / KNOWN_HEIGHT
							print 'focalLength'
							print focalLength
							
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
								
								if convex_hull_pointing_up(c):
									if (self.__debug_on == 1):
										cv2.drawContours(frame,[c],0,(0,255,0),3)
										cv2.putText(frame, "cone", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
									object_properties['name'] = 'CONE';
									object_properties['position'] = (cX, cY)
									marker = cv2.minAreaRect(c)
									object_properties['distance'] = (32.0 * 0.51)/disp[cY, cX]
									objects_list.append(object_properties.copy())
								else:
									if (self.__debug_on == 1):
										cv2.drawContours(frame,[c],0,(0,255,0),3)
										cv2.putText(frame, "bottle_orange", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
									object_properties['name'] = 'BOTTLE_'+self.__color_names[i];
									object_properties['position'] = (cX, cY)
									marker = cv2.minAreaRect(c)
									object_properties['distance'] = (32.0 * 0.51)/disp[cY, cX]
									objects_list.append(object_properties.copy())
									
								

		# show the frame to our screen
		if (self.__debug_on == 1):
			cv2.imshow("Frame", frame)
			cv2.waitKey(100)
		return objects_list
		
 