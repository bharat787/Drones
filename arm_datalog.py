#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright Mihir Vinay Kulkarni
arm_datalog.py: GUIDED mode

Demonstrates how to arm and takeoff, hover and land in copter mode
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import time
import math
import argparse
import csv
parser = argparse.ArgumentParser(description = 'Get Required data from the user')
parser.add_argument('--connect', help="PORT_NO")
parser.add_argument('--filename', help = "Filename for datalogging")
args = parser.parse_args()

connection_string = args.connect
filename = args.filename
print("Connecting to... % s" % connection_string)
vehicle = connect(connection_string, wait_ready = True, baud = 57600)
print("Connected")

sample_number, logstate, csvfile = 0, 0, None
fields = ['mode', 'altitude', 'yaw', 'pitch', 'roll', 'latitude', 'longitude', 'timestamp']
csvfile = open(filename, 'w')
writer = csv.DictWriter(csvfile, fieldnames=fields)
writer.writeheader()
init_time = time.time()


def datalogger():
    #print("Initiating logging:")
    #while vehicle.armed == False:
	if vehicle.armed == True:
		alti = vehicle.location.global_relative_frame.alt
		yaw , pitch, roll = vehicle.attitude.yaw*180/math.pi, vehicle.attitude.pitch*180/math.pi, vehicle.attitude.roll*180/math.pi
		mode = vehicle.mode.name
		print("mode=",mode, "altitude=", alti, "ypr=",yaw, pitch, roll)
		currentLocation = vehicle.location.global_relative_frame
		lat = currentLocation.lat
		lon = currentLocation.lon
		home = vehicle.home_location
		writer.writerow({'mode': mode, 'altitude': alti, 'yaw': yaw, 'pitch': pitch, 'roll': roll, 'latitude': lat, 'longitude': lon,'timestamp': -init_time+time.time()})

def arm_and_takeoff(aTargetAlt):
	print("Basic Prearm checks..dont touch!!")
	print("Waiting for vehicle to initialize")
	print("Arming Motors..")
	# Copter should arm in Guided-mode
	vehicle.mode = VehicleMode("GUIDED")
	time.sleep(4)
	vehicle.armed = True
	time.sleep(3)
	#vehicle.flush()
	while vehicle.armed == False:
		vehicle.armed = True
	print("Is Armed:% s"% vehicle.armed)
	time.sleep(3)
	print("Taking Off..")
	vehicle.simple_takeoff(aTargetAlt)

	while True:
		print("Altitude: ",vehicle.location.global_relative_frame.alt)
		#print(vehicle.attitude.yaw*180/math.pi, vehicle.attitude.pitch*180/math.pi, vehicle.attitude.roll*180/math.pi)
		#Break and return from function just below target altitude.
		datalogger()
		if vehicle.location.global_relative_frame.alt>=(aTargetAlt):
			print("Reached Target Altitude..")
			break
		time.sleep(0.2)
arm_and_takeoff(5)
print("REACHED HEIGHT")
init = time.time()
while time.time() - init < 20:
	datalogger()
	time.sleep(0.2)
vehicle.mode = VehicleMode("LAND")
while vehicle.armed ==True:
	datalogger()
	time.sleep(0.2)
vehicle.close()
