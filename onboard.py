from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import rospy
import time
import math
import csv

from std_msgs.msg import *
from geometry_msgs.msg import *

connection_string = '/dev/ttyACM1'
filename = 'log.csv'
print("Connecting to... % s" % connection_string)
vehicle = connect(connection_string, wait_ready = True, baud = 115200)
print("Connected")
#time.sleep(10)
sample_number, logstate, csvfile = 0, 0, None
fields = ['mode', 'altitude', 'yaw', 'pitch', 'roll', 'latitude', 'longitude', 'timestamp']
csvfile = open(filename, 'w')
writer = csv.DictWriter(csvfile, fieldnames=fields)
writer.writeheader()
init_time = time.time()

currentLocation = LocationGlobalRelative
target = Point()
ID = 1

rospy.init_node("onboard")
rate = rospy.Rate(10)
loc = PointStamped()
loc.header.seq = ID
home = vehicle.location.global_relative_frame


def datalogger():
	if vehicle.armed == True:
		alti = vehicle.location.global_relative_frame.alt
		yaw , pitch, roll = vehicle.attitude.yaw*180/math.pi, vehicle.attitude.pitch*180/math.pi, vehicle.attitude.roll*180/math.pi
		mode = vehicle.mode.name
		# print("mode=",mode, "altitude=", alti, "ypr=",yaw, pitch, roll)
		lat = currentLocation.lat
		lon = currentLocation.lon
		home = vehicle.home_location
		writer.writerow({'mode': mode, 'altitude': alti, 'yaw': yaw, 'pitch': pitch, 'roll': roll, 'latitude': lat, 'longitude': lon,'timestamp': -init_time+time.time()})


def set_target(msg):
	print("New Target")
	if int(msg.header.stamp) == ID:
		global target
		target.x = msg.point.x 
		target.y = msg.point.y
		target.z = msg.point.z


def arming(msg):
	print "in arm cb"
	print msg
	if msg.x == ID:
		print("ARMING")
		while vehicle.armed != msg.y:
			vehicle.armed = msg.y
		print("ARMED")


def forceLand(msg):
	print("LANDING")
	if msg.data == True:	
		vehicle.mode = VehicleMode("LAND")


targetSub = rospy.Subscriber('/setpoint',PointStamped, set_target)
armSub = rospy.Subscriber('/arm',Point,arming)
emergencySub = rospy.Subscriber('/emergency', Bool, forceLand)
sendLocation = rospy.Publisher('/gpose', PointStamped, queue_size = 5)
vehicle.mode = VehicleMode("GUIDED")
print "lets go"

try:
	while not rospy.is_shutdown():
		global currentLocation
		global loc
		global target
		# print "in loop"
		currentLocation = vehicle.location.global_relative_frame
		loc.point.x = currentLocation.lat
		loc.point.y = currentLocation.lon
		loc.point.z = currentLocation.alt
		datalogger()
		sendLocation.publish(loc)
		target_sp = LocationGlobalRelative(target.x, target.y, target.z)
		vehicle.simple_goto(target_sp)
		rate.sleep()
except rospy.ROSInterruptException:
	forceLand(True)