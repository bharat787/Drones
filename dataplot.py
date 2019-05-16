#/usr/bin/env/python
# -*- coding: utf-8 -*-

"""
© Copyright Mihir Vinay Kulkarni
arm_datalog.py: GUIDED mode


Program to extract data from the logged files and display the flight data graphically.
"""

import argparse
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from dronekit import LocationGlobalRelative, LocationGlobal


earth_radius = 6378137.0 #Radius of "spherical" earth

parser = argparse.ArgumentParser(description = 'Enter filename')
parser.add_argument('--filename', help = "Filename for data analysis")
args = parser.parse_args()
filename = args.filename
datalist = list()
with open(filename) as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        datalist.append(row)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

average_lat = sum(float(reading['latitude']) for reading in datalist)/len(datalist)
average_lon = sum(float(reading['longitude']) for reading in datalist)/len(datalist)
homeLocation = LocationGlobalRelative(average_lat, average_lon)
distance_to_avg = list()
altitude = list()
timestamp = list()
lat_list = list()
lon_list = list()
yaw = list()
pitch = list()
roll = list()
dNorth = list()
dEast = list()
for reading in datalist:
    currentLocation = LocationGlobalRelative(float(reading['latitude']), float(reading['longitude']))
    distance_to_avg.append(get_distance_metres(homeLocation, currentLocation))
    timestamp.append(float(reading['timestamp']))
    altitude.append(float(reading['altitude']))
    lat_list.append(float(reading['latitude']))
    lon_list.append(float(reading['longitude']))
    yaw.append(float(reading['yaw']))
    pitch.append(float(reading['pitch']))
    roll.append(float(reading['roll']))
    dNorth.append((float(reading['latitude']) - average_lat)*(math.pi/180)*earth_radius)
    dEast.append((float(reading['longitude']) - average_lon)*(math.pi/180)*(earth_radius*math.cos(math.pi*average_lat/180)))
    #print(distance_to_avg)
#plt.plot(distance_to_avg)
average_north = sum(dNorth)/len(dNorth)
average_east = sum(dEast)/len(dEast)
plt.figure('Flight Data')
plt.subplot(221)
plt.title('Altitude')
plt.plot(timestamp, altitude, 'b')
plt.ylabel("Altitude", fontsize = 10)
plt.subplot(222)
plt.title('Pitch(blue), Roll(red)')
plt.plot(timestamp, pitch, 'b')
plt.plot(timestamp, roll, 'r')
plt.ylabel("Angle in Degrees", fontsize = 10)
plt.subplot(223)
plt.title('GPS Deviation')
plt.plot(timestamp, distance_to_avg, 'b')
plt.ylabel("Distance from mean(m)", fontsize = 10)
plt.subplot(224)
plt.title('GPS Scatter')
plt.plot(dEast, dNorth, 'b*')
plt.plot(average_east, average_north, 'ro')
plt.xlabel("Deviation East(m)", fontsize = 10)
plt.ylabel("Deviation North(m)", fontsize = 10)
plt.show()

min_altitude = min(reading['altitude'] for reading in datalist)
max_altitude = max(reading['altitude'] for reading in datalist)
