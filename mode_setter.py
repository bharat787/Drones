###########################DEPENDENCIES###############################

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse

###########################FUNCTIONS##################################

def connectMyCopter():
	
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	
	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string,wait_ready=True)
	
	return vehicle

###########################MAIN exec###################################
vehicle = connectMyCopter()

while vehicle.is_armable!=True:
	print("Waiting for vehicle to become armable")
	time.sleep(1)
print("Vehicle is now armable")

vehicle.mode = VehicleMode('GUIDED')

while vehicle.mode!=('GUIDED'):
	print("waiting to enter into guided mode")
	time.sleep(1)
print("Now in guided mode")

vehicle.armed = True
while vehicle.armed==False:
	print("waiting to become armed")
	time.sleep(1)
print("the props are now spinning")


