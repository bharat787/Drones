import serial
import time 

ser = serial.Serial('/dev/ttyACM1', 9600)

def getdis() :
			
		
	#time.sleep(.1)
	b = ser.readline()
	try:		
		return int(b,10)
	except:
		return 0

while True:
	dis = getdis()	
	print(dis)
	
