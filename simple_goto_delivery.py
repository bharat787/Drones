from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException,Command
import time
import socket
import exceptions
import math
import argparse
import serial
import cv2
import numpy as np
from pymavlink import mavutil

ser = serial.Serial('/dev/ttyACM1', 9600) #--for port selection

gnd_speed =5 #--fixed speed when using set_velocity_body
#---------------------------------------------------------------------------

faces = cv2.CascadeClassifier("/home/bharat/Downloads/haarcascade.xml")

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX

def display():
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    face = faces.detectMultiScale(gray, 1.05, 5)
    for x, y, w, h in face:
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        if x<213:
            cv2.putText(img, 'RIGHT', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        elif 213 < x < 416:
            cv2.putText(img, 'CENTRE', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
             cv2.putText(img, 'LEFT', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow('img', img)
    # = cv2.waitKey(30) & 0xff
    #if k == 27:
     #       break  # 27 is for escape key
#---------------------------------------------------------------------------

def connectMyCopter():

    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    #-- uncomment when connrecting via telemetry
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
#--------------------------------------------------------------------------

def getDist():
    b = ser.readline()
    #reading = b.decode('utf-8')
    try:
        return int(b, 10) # 10 means decimal format
    except:
        return 100
#-------------------------------------------------------------------------

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
#-----------------------------------------------------------------------


#-- uncomment for user input of location
#loc = [float(x) for x in raw_input("What are the lat, long, alt of detination ").split(',')]
#loc = [28.468995493868242,77.53812865002897]
loc = [28.46951466793185,77.53698735091803]
vehicle = connectMyCopter()
arm_and_takeoff(5)

pt1 = LocationGlobalRelative(loc[0], loc[1], 10)
vehicle.simple_goto(pt1)

while True:
    d = getDist()
    print(d)
    if d< 20:
        print('collsion')
        set_velocity_body(vehicle, 0, gnd_speed, 0)
        # time.sleep(2)
        vehicle.simple_goto(pt1)
    #else:
    #     display()

    print vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon
    

    if round(vehicle.location.global_relative_frame.lat,5) == 28.46899 and round(vehicle.location.global_frame.lon,5) == 77.53813:
        print 'reached'
        break 
    else:
        print 'not reached'



'''
for i in range(30):
    print("We can run commands")
    time.sleep(1)

'''