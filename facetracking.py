import cv2
import numpy as np
import time
import socket
import exceptions
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil


gnd_speed = 5 # [m/s]

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
	
	return vehicle
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
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
    
faces = cv2.CascadeClassifier("/home/bharat/Downloads/haarcascade.xml")

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX
initSize = 36000

def faceTracking():

	while True:
		ret, img = cap.read()
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		face = faces.detectMultiScale(gray, 1.05, 5)
		for x, y, w, h in face:
		    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
		    if x<213:
		        cv2.putText(img, 'RIGHT', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			set_velocity_body(vehicle, 0, gnd_speed, 0)
		    elif 213 < x < 416:
		        cv2.putText(img, 'CENTRE', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			set_velocity_body(vehicle, gnd_speed, 0, 0)
		    else:
		        cv2.putText(img, 'LEFT', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
			set_velocity_body(vehicle, 0, -gnd_speed, 0)

		    print(w*h)
		    print(x, x+w)

		    size1 = w*h
		    #time.sleep(.1)
		    size2 = w*h
		    if size2 < 33000:
		        cv2.putText(img, 'GOING FAR', (0, 240), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
		    elif 330000 < size1 < 37000:
		        cv2.putText(img, 'NO CHANGE', (0, 240), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
		    else:
		        cv2.putText(img, 'COMING CLOSE', (0, 240), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


		cv2.imshow('img', img)
		k = cv2.waitKey(30) & 0xff
		if k == 27:
		    break  # 27 is for escape key

	cap.release()
	cv2.destroyAllWindows()

vehicle = connectMyCopter()
arm_and_takeoff(10)
faceTracking()

