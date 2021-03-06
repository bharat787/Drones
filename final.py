import multiprocessing
import cv2, time
import serial
import socket
import exceptions
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from multiprocessing import Value


#ser = serial.Serial('/dev/ttyACM1', 9600)                                            # Serial port connection for comms b/w sensor and USB

font = cv2.FONT_HERSHEY_SIMPLEX

video = cv2.VideoCapture(0)                                                           # Start camera

face_cascade = cv2.CascadeClassifier("/home/bharat/Downloads/haarcascade.xml")        # face detection xml file

gnd_speed = 5                                                                         # ground speed for drone

def connectMyCopter():                                                                # connecting drone to computer
	
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

def cv():                                                                             # face detection function

    while True:
    
        check, frame = video.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=5)
    
        for x, y, w, h in faces:
            img = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            if x<213:
                cv2.putText(img, 'RIGHT', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                direc.value = 1

            elif 213 < x < 416:
                cv2.putText(img, 'CENTRE', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                direc.value = 2
                
            else:
                cv2.putText(img, 'LEFT', (x, y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                direc.value = 3
            
        cv2.imshow("face", frame)
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        
    video.release()

    cv2.destroyAllWindows()

#-- Define arm and takeoff
def arm_and_takeoff(altitude):                                                        # arm and takeoff the drone to predefined altitude (5 meters) 

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: 
       time.sleep(1)

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
def set_velocity_body(vehicle, vx, vy, vz):                                           # send direction to drone 
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


def collision():                                                                      # detect collision

    return 10
    '''
    #while True:
        b = ser.readline()
        try:		
            return int(b,10) # 10 means decimal format
            #distance = int(b, 10)
        except:
            return 0
            #distance = 0
    '''
    #eturn 40

if __name__ == "__main__":
    
    direc = Value('d', 0)                                                             # initializing direction value 0-> no val 1->right 2->centre 3->left

    p1 = multiprocessing.Process(target=cv)                                           # assigning parallel processes to detect faces and collision simultaneously
    p2 = multiprocessing.Process(target=collision)


    p1.start()
    p2.start()

    loc = [28.46951466793185,77.53698735091803]                                       # vehicle home location
    vehicle = connectMyCopter()
    arm_and_takeoff(5)

    pt1 = LocationGlobalRelative(loc[0], loc[1], 10)
    vehicle.simple_goto(pt1)

    while True:
        print("the direction i'm receiving is " ,direc.value)
        
        d = collision()
        
        if d< 20:
            print('collsion')
            
            set_velocity_body(vehicle, 0, gnd_speed, 0)
            
            vehicle.simple_goto(pt1)

        print vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon
        

        if round(vehicle.location.global_relative_frame.lat,5) == 28.46899 and round(vehicle.location.global_frame.lon,5) == 77.53813:
            print 'reached'
            break 
        else:
            print 'not reached'
       