###########################DEPENDENCIES###############################

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

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
	#vehicle = connect('127.0.0.1:14550', wait_ready=True)
	
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

def local_ned(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def global_ned(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_LOCAL_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
		

###########################MAIN exec###################################
vehicle = connectMyCopter()

arm_and_takeoff(10)

counter = 0
while counter<5:
	local_ned(5,0,0)
	time.sleep(1)
	print("Moving NORTH relative to front of drone")
	counter = counter + 1

time.sleep(2)

counter = 0
while counter<5:
	global_ned(0,-5,0)
	time.sleep(1)
	print("Moving TRUE WEST relative to front of drone")
	counter = counter + 1

time.sleep(2)

counter = 0
while counter<5:
	local_ned(-5,0,0)
	time.sleep(1)
	print("Moving SOUTH relative to front of drone")
	counter = counter + 1

time.sleep(2)

counter = 0
while counter<5:
	global_ned(0,5,0)
	time.sleep(1)
	print("Moving TRUE EAST relative to front of drone")
	counter = counter + 1

while True:
	time.sleep(1)


