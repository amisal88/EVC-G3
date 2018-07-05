from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from vision import *
import serial


def main():
	Object_detector_inst = Object_detector(0)
	ser = serial.Serial('/dev/ttyUSB0', 9600)
	while True:
		#s = input("Please enter a key to capture images")
                for i in range(0,5):
                    object_list = Object_detector_inst.objects_process();
                #object_list = Object_detector_inst.stereo_objects_process();
##                    for d in object_list:
##                        print("\n")
##                        for i in d:
##                            print i, d[i]
		object = object_list[0]
		if(object['name'] == 'cone'):
                    ser_msg = 'XYmove('+str(object['position'][0])+','+str(object['position'][1])+')'
                    print 'ser_msg'
                    print ser_msg
                    ser.write(ser_msg)
                    while(True):
                        recv = ser.read(2)
                        print recv
                        if (recv == 'OK'):
                            break
                    break
                    
                          
                        
##                QR_code = Object_detector_inst.qr_process();
##                print QR_code
		#names = Object_detector_inst.faces_process();
                #print names
##                
                        
		

if __name__ == "__main__":
    main()