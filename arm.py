#/usr/bin/env python 
# -*- coding: utf-8 -*-
from dronekit import connect,VehicleMode,LocationGlobalRelative,Vehicle
from pymavlink import mavutil
import time
import math
# Receiving from command line
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect',help="PORT_NO")
args = parser.parse_args()

# Connecting to the vehicle
connection_string = args.connect
print("Connecting to...% s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
print("Connected")
#Function to arm and takeoff to a specified altitude
def arm_and_takeoff(aTargetAlt):
	print("Basic Prearm checks..dont touch!!")
	print("Waiting for vehicle to initialize")
	time.sleep(4)
	print("Arming Motors..")
	# Copter should arm in Guided-mode
	vehicle.mode = VehicleMode("GUIDED")
	#time.sleep(4)
	vehicle.armed = True
	time.sleep(3)
	#vehicle.flush()
	print("Is Armed:% s"% vehicle.armed)
	time.sleep(1)
	print("Taking Off..")
	vehicle.simple_takeoff(aTargetAlt)

	while True:
		print("Altitude: ",vehicle.location.global_relative_frame.alt)
		print(vehicle.attitude.yaw*180/math.pi, vehicle.attitude.pitch*180/math.pi, vehicle.attitude.roll*180/math.pi)
		#Break and return from function just below target altitude.
		if vehicle.location.global_relative_frame.alt>=(aTargetAlt-1):
			print("Reached Target Altitude..")
			break
		time.sleep(1)


arm_and_takeoff(5)
#vehicle.mode = VehicleMode("LOITER")
time.sleep(10)
vehicle.mode = VehicleMode("LAND")
#vehicle.close()
