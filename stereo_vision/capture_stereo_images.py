import RPi.GPIO as gp
import os
from picamera import PiCamera
from time import sleep

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

def main():
	
	t = 0
	camera = PiCamera()
	camera.resolution = (1024, 768)
	while True:
		s = input("Please enter a key to capture images")
		# take an image from channel 1
		gp.output(7, False)
		gp.output(11, False)
		gp.output(12, True)
		
		camera.start_preview()
		sleep(2)
		camera.capture('./chessboard-R'+str(t)+'.png')
		camera.stop_preview()
		
		# take an image from channel 3
		gp.output(7, False)
		gp.output(11, True)
		gp.output(12, False)
		
		camera.start_preview()
		sleep(2)
		camera.capture('./chessboard-L'+str(t)+'.png')
		camera.stop_preview()
		
		t = t + 1
		

def capture(cam):
    cmd = "raspistill -o capture_%d.jpg" % cam
    os.system(cmd)

if __name__ == "__main__":
    main()

    gp.output(7, False)
    gp.output(11, False)
    gp.output(12, True)