#/usr/bin/env python
# -*- coding: utf-8 -*-
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

def datalogger():
    print("Initiating logging:")
    while vehicle.armed == False:
        vehicle.armed = True
        time.sleep(2)
        vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(5)        
        vehicle.mode = VehicleMode("POSHOLD")
        init_time = time.time();
    while vehicle.armed == True:    	
        alti = vehicle.location.global_relative_frame.alt
        yaw , pitch, roll = vehicle.attitude.yaw*180/math.pi, vehicle.attitude.pitch*180/math.pi, vehicle.attitude.roll*180/math.pi
        mode = vehicle.mode.name
        print("mode=",mode, "altitude=", alti, "ypr=",yaw, pitch, roll)
        currentLocation = vehicle.location.global_relative_frame
        lat = currentLocation.lat
        lon = currentLocation.lon
        writer.writerow({'mode': mode, 'altitude': alti, 'yaw': yaw, 'pitch': pitch, 'roll': roll, 'latitude': lat, 'longitude': lon, 'timestamp': -init_time+time.time()})
        time.sleep(0.05)
    if csvfile and not csvfile.closed:
		csvfile.close()

datalogger()
time.sleep(5)
vehicle.close()
