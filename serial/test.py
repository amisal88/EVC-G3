import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

ser.write('rotate(1.0)')

while(ser.readline()!='OK'):
	print 'waiting ... \n'

print 'done\n'