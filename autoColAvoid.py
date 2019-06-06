###########################DEPENDENCIES###############################

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException,Command
import time
import socket
import exceptions
import math
import argparse
import serial
from pymavlink import mavutil
ser = serial.Serial('/dev/ttyACM1', 9600)

gnd_speed = 5

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

#-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
   
def getDist():
	b = ser.readline()
	try:		
		return int(b,10)
	except:
		return 0
 
#-- Collsion detection
def collision():
	dist  = getDist()
	print(dist)
	if dist <= 60:
		set_velocity_body(vehicle,-gnd_speed, 0, 0)


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
	#print("Drone is executing mission, but we can still run the code")
	collision()
	time.sleep(.2)




