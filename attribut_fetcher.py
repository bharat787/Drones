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

vehicle.wait_ready('autopilot_version')
print('Autopilot version is: %s'%vehicle.version)

print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

print('position is: %s'%vehicle.location.global_relative_frame)

print('attitude is: %s'%vehicle.attitude)

print('velocity is: %s'%vehicle.velocity) #north, east, down

print('is the drone armable: %s'%vehicle.is_armable)

print('ground speed: %s'%vehicle.groundspeed)

print('Mode: %s'%vehicle.mode.name)

print('Armed: %s'%vehicle.armed)

print('ekf ok: %s'%vehicle.ekf_ok)

vehicle.close()
