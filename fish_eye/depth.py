# Package importation

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import RPi.GPIO as gp
import time
import pickle

# Filtering
kernel= np.ones((3,3),np.uint8)

def coords_mouse_disp(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDBLCLK:
		#print x,y,disp[y,x],filteredImg[y,x]
		average=0
		for u in range (-1,2):
			for v in range (-1,2):
				average += disp[y+u,x+v]
		average=average/9
		#Distance= -593.97*average**(3) + 1506.8*average**(2) - 1373.1*average + 522.06
		Distance = (32*0.51)/average;
		Distance= np.around(Distance,decimals=10)
		print('Distance: '+ str(Distance)+' cm')
		

#********************************************
#***** Calibrate the Cameras for Stereo *****
#********************************************

#*******************************************
#***** Parameters for the StereoVision *****
#*******************************************
# SGBM Parameters -----------------
window_size = 3					 # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

min_disp = 0
num_disp = 160-min_disp

left_matcher = cv2.StereoSGBM_create(
	minDisparity=min_disp,
	numDisparities=num_disp,			 # max_disp has to be dividable by 16 f. E. HH 192, 256
	blockSize=5,
	P1=8 * 3 * window_size ** 2,	# wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
	P2=32 * 3 * window_size ** 2,
	disp12MaxDiff=1,
	uniquenessRatio=15,
	speckleWindowSize=0,
	speckleRange=2,
	preFilterCap=63,
	mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

# FILTER Parameters
lmbda = 80000
sigma = 1.2
visual_multiplier = 1.0
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

#*************************************
#***** Starting the StereoVision *****
#*************************************

# Call the two cameras
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

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera)

# allow the camera to warmup
time.sleep(0.1)

DIM=(640, 480)
KL=np.array([[310.89487794241705, 0.0, 310.4701490632405], [0.0, 311.6045360428195, 217.332401108332], [0.0, 0.0, 1.0]])
DL=np.array([[-0.02678943147680496], [0.07199621192297553], [-0.17537795442931486], [0.1224981141497315]])
KR=np.array([[311.9993342853497, 0.0, 288.39779549515345], [0.0, 312.9238338162564, 233.89810329394894], [0.0, 0.0, 1.0]])
DR=np.array([[-0.035125623121583065], [0.0721407010760906], [-0.1341791663672435], [0.07146034215266592]])

while True:
	# Start Reading Camera images
	gp.output(7, False)
	gp.output(11, False)
	gp.output(12, True)
	time.sleep(0.1)
	camera.capture(rawCapture, format="bgr")
	frameR = rawCapture.array
	rawCapture.truncate(0)
	gp.output(7, False)
	gp.output(11, True)
	gp.output(12, False)
	time.sleep(0.1)
	camera.capture(rawCapture, format="bgr")
	frameL = rawCapture.array
	rawCapture.truncate(0)

	# Rectify the images on rotation and alignement
	map1L, map2L = cv2.fisheye.initUndistortRectifyMap(KR, DR, np.eye(3), KR, DIM, cv2.CV_16SC2)
	Left_nice = cv2.remap(frameL, map1L, map2L, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	
	map1R, map2R = cv2.fisheye.initUndistortRectifyMap(KR, DR, np.eye(3), KR, DIM, cv2.CV_16SC2)
	Right_nice = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

	cv2.imshow('Right_nice', Right_nice)
	cv2.imshow('Left_nice', Left_nice)

	k = cv2.waitKey(100)
##	# Draw Red lines
##	for line in range(0, int(Right_nice.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
##		Left_nice[line*20,:]= (0,0,255)
##		Right_nice[line*20,:]= (0,0,255)
##
##	for line in range(0, int(frameR.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
##		frameL[line*20,:]= (0,255,0)
##		frameR[line*20,:]= (0,255,0)	
		
	# Show the Undistorted images
	#cv2.imshow('Both Images', np.hstack([Left_nice, Right_nice]))
	#cv2.imshow('Normal', np.hstack([frameL, frameR]))

	# Convert from color(BGR) to gray
	
	imgR= cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
	imgL= cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)

	displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
	dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
	displ = np.int16(displ)
	dispr = np.int16(dispr)
	filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
	
	filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
	filteredImg = np.uint8(filteredImg)
	cv2.imshow('Disparity Map', filteredImg)
	disp= ((displ.astype(np.float32)/ 16)-min_disp)/num_disp
	
	k = cv2.waitKey(100)

	# Mouse click
	cv2.setMouseCallback("Disparity Map",coords_mouse_disp)
	
# Save excel
##wb.save("data4.xlsx")

# Release the Cameras
cv2.destroyAllWindows()


