import serial
import time
ser = serial.Serial('/dev/ttyUSB0', 9600)

time.sleep(2);
#ser.write('XYmove(100,200)')
ser.write('deliver(1)')

while(True):
        recv = ser.readline()
        print recv
##	if(recv == 'OK'):
##            break

print 'done\n'