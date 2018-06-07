import RPi.GPIO as gp
import os
from picamera import PiCamera
from time import sleep

Num_of_objects = 100

object_properties = {'valid' : 'bottle' , 'name' : 'bottle', 'probability' : 0.0, 'position' : (0,0), 'stdev_p' : 0, 'rotation' : 0.0 , 'stdev_r': 0.0}

objects_list = [object_properties for _ in range(Num_of_objects)]

