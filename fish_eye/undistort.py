# Package importation
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
import RPi.GPIO as gp
import time
import pickle

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


##DIM=(640, 480)
##KL=np.array([[310.89487794241705, 0.0, 310.4701490632405], [0.0, 311.6045360428195, 217.332401108332], [0.0, 0.0, 1.0]])
##DL=np.array([[-0.02678943147680496], [0.07199621192297553], [-0.17537795442931486], [0.1224981141497315]])
##KR=np.array([[311.9993342853497, 0.0, 288.39779549515345], [0.0, 312.9238338162564, 233.89810329394894], [0.0, 0.0, 1.0]])
##DR=np.array([[-0.035125623121583065], [0.0721407010760906], [-0.1341791663672435], [0.07146034215266592]])



DIM=(2560, 720)
KL=np.array([[1312.7561575169264, 0.0, 1243.9785618233964], [0.0, 1319.9570065829275, 181.55794724721528], [0.0, 0.0, 1.0]])
DL=np.array([[-0.027921856911960495], [-0.5309936524548845], [13.887341952399481], [-71.30274940957545]])
KR=np.array([[1267.6200754096494, 0.0, 1175.8229642050703], [0.0, 1277.097182371003, 263.6310009704831], [0.0, 0.0, 1.0]])
DR=np.array([[0.13028905994074766], [-5.499980327032473], [66.23905695079091], [-281.539690831334]])# initialize the camera and grab a reference to the raw camera capture

camera = PiCamera()
camera.resolution = (2560, 720)
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)


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
	
	map1L, map2L = cv2.fisheye.initUndistortRectifyMap(KR, DR, np.eye(3), KR, DIM, cv2.CV_16SC2)
	Left_nice = cv2.remap(frameL, map1L, map2L, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	
	map1R, map2R = cv2.fisheye.initUndistortRectifyMap(KR, DR, np.eye(3), KR, DIM, cv2.CV_16SC2)
	Right_nice = cv2.remap(frameR, map1R, map2R, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
	# Rectify the images on rotation and alignement
	

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



