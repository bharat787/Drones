###########################DEPENDENCIES###############################

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException,Command
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

###########################MAIN exec###################################
vehicle = connectMyCopter()
print("HOME!!!!%s " % vehicle.home_location)
#vehicle.home_location=vehicle.location.global_frame
#vehicle.location.global_relative_frame = (15.393963,73.883284,0,180)

wpHome = vehicle.location.global_relative_frame

cmd1=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,0,0,0,0,wpHome.lat,wpHome.lon,wpHome.alt)
cmd2=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,0,0,0,0,44.501375,-88.062645,15)
cmd3=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,0,0,0,0,44.501746,-88.062242,10)
cmd4=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0,0,0,0,0,0,0,0)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

cmds.clear()

cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)

vehicle.commands.upload()

arm_and_takeoff(10)

print("after arm and takeoff")
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode !="AUTO":
	time.sleep(.2)

while vehicle.location.global_relative_frame.alt>2:
	print("Drone is executing mission, but we can still run the code")
	time.sleep(.2)




