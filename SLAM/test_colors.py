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

camera = PiCamera()
camera.resolution = (2560, 720)
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

	cv2.imshow('imgR',frameR)
	cv2.imshow('imgL',frameL)
	cv2.waitKey(10)
	
	yn = raw_input('Save the image? y/n ')
	if yn == 'y':
		t= str(i)
		print('Saved'+t)
		cv2.imwrite('test-R'+t+'.png',frameR) # Save the image in the file where this Programm is located
		cv2.imwrite('test-L'+t+'.png',frameL)
		i=i+1
	else:
		print('canceled')
	
	




