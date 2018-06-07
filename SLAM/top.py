from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
from vision import *

def main():
	Object_detector_inst = Object_detector(1, "field.mp4")
	while True:
		#s = input("Please enter a key to capture images")
		object_list = Object_detector_inst.process();
		
		for d in object_list:
                    print("\n")
                    for i in d:
                        print i, d[i]
                        
		

if __name__ == "__main__":
    main()
