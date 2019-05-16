#/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright Mihir Vinay Kulkarni
arm_datalog.py: GUIDED mode

Demonstrates how to arm and takeoff, follow a sqaure trajectory and land in Copter mode.
Datalogging is done at all steps.
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
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

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


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

def arm_and_takeoff(aTargetAlt):
	print("Basic Prearm checks..dont touch!!")
	print("Waiting for vehicle to initialize")
	time.sleep(4)
	print("Arming Motors..")
	# Copter should arm in Guided-mode
	vehicle.mode = VehicleMode("GUIDED")
	#time.sleep(4)
	vehicle.armed = True
	time.sleep(2)
	#vehicle.flush()
	while vehicle.armed == False:
		vehicle.armed = True
	print("Is Armed:% s"% vehicle.armed)
	time.sleep(1)
	print("Taking Off..")
	vehicle.simple_takeoff(aTargetAlt)

	while True:
		print("Altitude: ",vehicle.location.global_relative_frame.alt)
		#print(vehicle.attitude.yaw*180/math.pi, vehicle.attitude.pitch*180/math.pi, vehicle.attitude.roll*180/math.pi)
		#Break and return from function just below target altitude.
		datalogger()
		if vehicle.location.global_relative_frame.alt>=(aTargetAlt-0.05):
			print("Reached Target Altitude..")
			break
		time.sleep(0.2)


"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in 
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in 
    MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use Vehicle.simple_goto (default) or 
    goto_position_target_global_int to travel to a specific position in metres 
    North and East from the current location. 
    This method reports distance to the destination.
"""


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print "Distance to target: ", remainingDistance
        datalogger()
        if remainingDistance<=0.7: #Just below target, in case of undershoot.
            print "Reached target"
            break;
        time.sleep(0.2)

arm_and_takeoff(5)
#vehicle.mode = VehicleMode("LOITER")
print("REACHED HEIGHT")
"""
Fly a triangular path using the standard Vehicle.simple_goto() method.

The method is called indirectly via a custom "goto" that allows the target position to be
specified as a distance in metres (North/East) from the current position, and which reports
the distance-to-target.
"""	
print("TRIANGLE path using standard Vehicle.simple_goto()")

print("Set groundspeed to 5m/s.")
vehicle.groundspeed=3

goto(5, 0)
init = time.time()
while time.time() - init < 2:
	datalogger()
	time.sleep(0.2)
goto(0, 5)
init = time.time()
while time.time() - init < 2:
	datalogger()
	time.sleep(0.2)
goto(-5, 0)
init = time.time()
while time.time() - init < 2:
	datalogger()
	time.sleep(0.2)

goto(0, -5)
init = time.time()
while time.time() - init < 2:
	datalogger()
	time.sleep(0.2)


"""
The example is completing. LAND at current location.
"""

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")


#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
sitl = None
# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()






'''
print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(20)

print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
time.sleep(20)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

init = time.time()
while time.time() - init < 10:
	datalogger()
	time.sleep(0.2)
goto(10,0,0)
while time.time() - init < 20:
	datalogger()
	time.sleep(0.2)

goto(0,10,0)
while time.time() - init < 20:
	datalogger()
	time.sleep(0.2)
goto(-10,0,0)
while time.time() - init < 20:
	datalogger()
	time.sleep(0.2)
goto(0,-10,0)
while time.time() - init < 20:
	datalogger()
	time.sleep(0.2)

vehicle.mode = VehicleMode("LAND")

while vehicle.armed ==True:
	datalogger()
	time.sleep(0.2)
vehicle.close()
'''
