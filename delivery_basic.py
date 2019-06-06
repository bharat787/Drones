from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException,Command
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

#---------------------------------------------------------------------------
def connectMyCopter():
	
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect
	'''
	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	vehicle = connect(connection_string,wait_ready=True, baud=57600)
	'''

	if not connection_string:
		import dronekit_sitl
		from dronekit_sitl import SITL
		
		sitl = SITL()
		sitl.download('copter', '3.3', verbose=True)
		sitl_args = ['-I0', '--model', 'quad', '--home=28.468269,77.53859,200,353']
		sitl.launch(sitl_args, verbose=True, await_ready=False, restart=True)
		
		#vehicle = connect(connection_string,wait_ready=True, baud=57600)
	vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
	return vehicle
#---------------------------------------------------------------------------

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
#---------------------------------------------------------------------------

loc = [float(x) for x in raw_input("What are the lat, long, alt of detination ").split(',')]

vehicle = connectMyCopter()

wpHome = vehicle.location.global_relative_frame

cmd1=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,0,0,0,0,wpHome.lat,wpHome.lon,wpHome.alt)
cmd2=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,0,0,0,0,loc[0],loc[1],loc[2])
cmd3=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0,0,0,0,0,0,0,0)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

cmds.clear()

cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)

vehicle.commands.upload()
vehicle.groundspeed = 1000
arm_and_takeoff(5)

print("after arm and takeoff")
vehicle.mode = VehicleMode("AUTO")
while vehicle.mode !="AUTO":
	time.sleep(.2)

while vehicle.location.global_relative_frame.alt>2:
	print("Drone is executing mission, but we can still run the code")
	time.sleep(.2)
