from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from vision import *

def main():
	Object_detector_inst = Object_detector(1)
	while True:
		#s = input("Please enter a key to capture images")
		object_list = Object_detector_inst.objects_process();
                #object_list = Object_detector_inst.stereo_objects_process();
		
		for d in object_list:
                    print("\n")
                    for i in d:
                        print i, d[i]
                        
##                QR_code = Object_detector_inst.qr_process();
##                print QR_code
		#names = Object_detector_inst.faces_process();
                #print names
##                
                        
		

if __name__ == "__main__":
    main()