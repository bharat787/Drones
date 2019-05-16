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
def arm_and_takeoff(targetHeight):
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

	vehicle.simple_takeoff(targetHeight) #in meters

	while True:
		print("Current altitude is: %d"%vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
			break
		time.sleep(1)
	print("target altitude reached!!")
	return None
def get_distance_meters(targetLocation, currentLocation):
	dLat = targetLocation.lat - currentLocation.lat
	dLon = targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def goto(targetLocation):
	print("in goto")
	#vehicle.location.global_relative_frame = (15.392900,73.882725,0)
	distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)
	while vehicle.mode.name=='GUIDED':
		currentDistance= get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
		print("current location is %s"%vehicle.location.global_relative_frame)
		if currentDistance<0.1*distanceToTargetLocation:
			print("target location reached")
			time.sleep(2)
			break
		time.sleep(1)
	return None
###########################MAIN exec###################################

wp1 = LocationGlobalRelative(44.4998779, -88.049727,10)

vehicle = connectMyCopter()

arm_and_takeoff(10)

goto(wp1)

vehicle.mode = VehicleMode('LAND')
while vehicle.mode !='LAND':
	print("waiting to enter in land mode")
	time.sleep(1)
print("vehicle in land mode")

while True:
	time.sleep(1)

