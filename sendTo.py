import re
import os
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
import time
import socket
import exceptions
import math
import argparse
import serial
from pymavlink import mavutil

#-- uncomment when using telemetry
#  ser = serial.Serial('/dev/ttyACM1', 9600)  # --for port selection

gnd_speed = 5  # --fixed speed when using set_velocity_body
# ---------------------------------------------------------------------------

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    # -- uncomment when connrecting via telemetry
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
        sitl_args = ['-I0', '--model', 'quad', '--home=28.468772344967554,77.53801532669809,200,353']
        sitl.launch(sitl_args, verbose=True, await_ready=False, restart=True)

        # vehicle = connect(connection_string,wait_ready=True, baud=57600)
    vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
    return vehicle


# ---------------------------------------------------------------------------

def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = VehicleMode('GUIDED')

    while vehicle.mode != ('GUIDED'):
        print("waiting to enter into guided mode")
        time.sleep(1)
    print("Now in guided mode")

    vehicle.armed = True
    while vehicle.armed == False:
        print("waiting to become armed")
        time.sleep(1)
    print("the props are now spinning")

    vehicle.simple_takeoff(targetHeight)  # in meters

    while True:
        print("Current altitude is: %d" % vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= .95 * targetHeight:
            break
        time.sleep(1)
    print("target altitude reached!!")
    return None


# --------------------------------------------------------------------------

#TODO get code from file
def getLocs():

    fname = '/home/bharat/Desktop/dk/Drones/values.txt'
    f = open(fname, 'r')
    locs = (f.read()).split(',')
    os.remove(fname)
    return locs


# -------------------------------------------------------------------------



# -- uncomment for user input of location
# loc = [float(x) for x in raw_input("What are the lat, long, alt of detination ").split(',')]
# loc = [28.468995493868242,77.53812865002897]
coordinates = getLocs()
vehicle = connectMyCopter()
arm_and_takeoff(5)
print("failure point breached")
pt1 = LocationGlobalRelative(float(coordinates[0]), float(coordinates[1]), 10)
vehicle.simple_goto(pt1)
time.sleep(600)
print("if you see this early, it didnt work")


