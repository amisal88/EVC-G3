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

print('Starting the Calibration just press the space bar to exit this part of the Programm\n')
print('Push (s) to save the image you want and push (c) to see next frame without saving the image')

i=0
C= False

# termination criteria
criteria =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all images
objpoints= []   # 3d points in real world space
imgpointsR= []  # 2d points in image plane   
imgpointsL= []
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera)

sleep(2)
k=0
while True:
	# take an image from channel 1
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
	
	grayR= cv2.cvtColor(frameR,cv2.COLOR_BGR2GRAY)
	grayL= cv2.cvtColor(frameL,cv2.COLOR_BGR2GRAY)

	# Find the chess board corners
	retR, cornersR = cv2.findChessboardCorners(grayR,(9,6),None)  # Define the number of chess corners we are looking for
	retL, cornersL = cv2.findChessboardCorners(grayL,(9,6),None)  # Left side
	cv2.imshow('imgR',frameR)
	cv2.imshow('imgL',frameL)

	k=k+1
	print k
	cv2.waitKey(100)
	# If found, add object points, image points (after refining them)
	if (True == retR) & (True == retL) & (False == C):
		objpoints.append(objp)
		corners2R= cv2.cornerSubPix(grayR,cornersR,(11,11),(-1,-1),criteria)	# Refining the Position
		imgpointsR.append(cornersR)
		corners2L= cv2.cornerSubPix(grayL,cornersL,(11,11),(-1,-1),criteria)
		imgpointsL.append(cornersL)

		# Draw and display the corners
		cv2.drawChessboardCorners(grayR,(9,6),corners2R,retR)
		cv2.drawChessboardCorners(grayL,(9,6),corners2L,retL)
		cv2.imshow('VideoR',grayR)
		cv2.imshow('VideoL',grayL)
		cv2.waitKey(100)
		yn = raw_input('Save the image? y/n ')
		if yn == 'y':
			t= str(i)
			print('Saved'+t)
			cv2.imwrite('chessboard-R'+t+'.png',frameR) # Save the image in the file where this Programm is located
			cv2.imwrite('chessboard-L'+t+'.png',frameL)
			i=i+1
		else:
			print('canceled')

	# End the Programme
	if cv2.waitKey(1) & 0xFF == ord(' '):   # Push the space bar and maintan to exit this Programm
		break

	# Release the Cameras
	#CamR.release()
	#CamL.release()



