#***********************
#**** Main Programm ****
#***********************


# Package importation
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import RPi.GPIO as gp
import time
import pickle
from openpyxl import Workbook # Used for writing data into an Excel file
#from sklearn.preprocessing import normalize

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
		Distance= -593.97*average**(3) + 1506.8*average**(2) - 1373.1*average + 522.06
		Distance= np.around(Distance*0.01,decimals=2)
		print('Distance: '+ str(Distance)+' m')
		
# This section has to be uncommented if you want to take mesurements and store them in the excel
##		ws.append([counterdist, average])
##		print('Measure at '+str(counterdist)+' cm, the dispasrity is ' + str(average))
##		if (counterdist <= 85):
##			counterdist += 3
##		elif(counterdist <= 120):
##			counterdist += 5
##		else:
##			counterdist += 10
##		print('Next distance to measure: '+str(counterdist)+'cm')

# Mouseclick callback
wb=Workbook()
ws=wb.active  

#*************************************************
#***** Parameters for Distortion Calibration *****
#*************************************************

# Termination criteria
criteria =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all images
objpoints= []   # 3d points in real world space
imgpointsR= []   # 2d points in image plane
imgpointsL= []

#*******************************************
#***** Parameters for the StereoVision *****
#*******************************************

# Create StereoSGBM and prepare all parameters
window_size = 3
min_disp = 2
num_disp = 130-min_disp
stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
	numDisparities = num_disp,
	blockSize = window_size,
	uniquenessRatio = 10,
	speckleWindowSize = 100,
	speckleRange = 32,
	disp12MaxDiff = 5,
	P1 = 8*3*window_size**2,
	P2 = 32*3*window_size**2)

# Used for the filtered image
stereoR=cv2.ximgproc.createRightMatcher(stereo) # Create another stereo for right this time

# WLS FILTER Parameters
lmbda = 80000
sigma = 1.8
visual_multiplier = 1.0
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

#*************************************
#***** Starting the StereoVision *****
#*************************************
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


file = open('Left_Stereo_Map_file.txt', 'r')
Left_Stereo_Map = pickle.load(file)
file.close()

file = open('Right_Stereo_Map_file.txt', 'r')
Right_Stereo_Map = pickle.load(file)
file.close()

print Left_Stereo_Map
print Right_Stereo_Map

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
	Left_nice= cv2.remap(frameL,Left_Stereo_Map[0],Left_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)  # Rectify the image using the kalibration parameters founds during the initialisation
	Right_nice= cv2.remap(frameR,Right_Stereo_Map[0],Right_Stereo_Map[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
	
	cv2.imshow('Right Nice', Right_nice)
	cv2.imshow('Left Nice', Left_nice)
	
	cv2.imshow('Right', frameR)
	cv2.imshow('Left', frameL)
	
	k = cv2.waitKey(100)
	# End the Programme
	if cv2.waitKey(1) & 0xFF == ord(' '):
		break
	
# Save excel
##wb.save("data4.xlsx")

# Release the Cameras
CamR.release()
CamL.release()
cv2.destroyAllWindows()


