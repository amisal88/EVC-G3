import RPi.GPIO as gp
import os
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import cv2
import numpy as np
import time

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
rawCapture = PiRGBArray(camera, size=(640, 480))

DIM=(640, 480)
KL=np.array([[310.89487794241705, 0.0, 310.4701490632405], [0.0, 311.6045360428195, 217.332401108332], [0.0, 0.0, 1.0]])
DL=np.array([[-0.02678943147680496], [0.07199621192297553], [-0.17537795442931486], [0.1224981141497315]])
KR=np.array([[311.9993342853497, 0.0, 288.39779549515345], [0.0, 312.9238338162564, 233.89810329394894], [0.0, 0.0, 1.0]])
DR=np.array([[-0.035125623121583065], [0.0721407010760906], [-0.1341791663672435], [0.07146034215266592]])
# allow the camera to warmup
time.sleep(2)
 
i = 0
# capture frames from the camera
while True:
	camera.capture(rawCapture, format="bgr")
	frameR = rawCapture.array
	rawCapture.truncate(0)
	
	map1R, map2R = cv2.fisheye.initUndistortRectifyMap(KR, DR, np.eye(3), KR, DIM, cv2.CV_16SC2)
	Right_nice = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
 
	# show the frame
	cv2.imshow("Frame", Right_nice)
	cv2.waitKey(100)
	yn = raw_input('Save the image? y/n ')
	if yn == 'y':
		t= str(i)
		cv2.imwrite('Image'+t+'.png',Right_nice) # Save the image in the file where this Programm is located
		i=i+1
		print('Saved')
	else:
		print('canceled')
