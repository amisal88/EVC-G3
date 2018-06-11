from picamera.array import PiRGBArray
from picamera import PiCamera
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import time
import cv2
import numpy as np
import RPi.GPIO as gp
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
						cv2.approxPolyDP(cnt, cv2.arcLength(cnt, True) * 0.02, True)
					if len(apprx) == 4:
						corner_cnts.append(apprx.reshape((4, -1)))
				if len(corner_cnts) == 3:
					cnts.append(corner_cnts)
					all_pts = np.array(corner_cnts).reshape(-1, 2)
					
					centers.append(np.mean(all_pts, 0))
	print centers
	print("\n")
	
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

file = open('Left_Stereo_Map_file.txt', 'r')
Left_Stereo_Map = pickle.load(file)
file.close()

file = open('Right_Stereo_Map_file.txt', 'r')
Right_Stereo_Map = pickle.load(file)
file.close()

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera)
 
# allow the camera to warmup
time.sleep(0.1)

while True:
	# grab an image from the camera
	camera.capture(rawCapture, format="bgr")
	image_in = rawCapture.array
	image= cv2.remap(image_in,Right_Stereo_Map[0],Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	result, corners = qr_code_outer_corners(image)
	#print corners
	qr_code_size = 100
	
	if result:
		if all((0, 0) < tuple(c) < (image.shape[1], image.shape[0]) for c in corners):
			rectified = rectify(image, corners, (qr_code_size, qr_code_size))
			
			cv2.circle(image, tuple(corners[0]), 15, (0, 255, 0), 2)
			cv2.circle(image, tuple(corners[1]), 15, (0, 0, 255), 2)
			cv2.circle(image, tuple(corners[2]), 15, (255, 0, 0), 2)
			cv2.circle(image, tuple(corners[3]), 15, (255, 255, 0), 2)
			image.flags.writeable = True
			image[0:qr_code_size, 0:qr_code_size] = rectified

	cv2.imshow('QR code detection', image)
	
	k = cv2.waitKey(100)
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	if k == 27:
		break

vs.release()
cv2.destroyAllWindows()


