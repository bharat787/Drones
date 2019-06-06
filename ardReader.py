import serial
import time

#set up serial
ser  = serial.Serial('/dev/ttyACM0', 9600)
#time.sleep(2)

#read and record data
#data = []
while True:
	#print(i)
	b = ser.readline()
	#string_n = b.decode()
	#string = string_n.rstrip()
	#flt = float(string)
	print int(b, 10)
	#data.append(flt)
	#time.sleep(0.1)

ser.closed()

#for line in data:
#	print(line)	
