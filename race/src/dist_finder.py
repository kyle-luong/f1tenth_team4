#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 2	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.65	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 20 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where    0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement

	
	i = int(math.floor((angle - data.angle_min) / data.angle_increment))
	print("I: ", i)
	if angle < data.angle_min or angle > data.angle_max:
		return float('inf')
	
	if i < 0 or i >= len(data.ranges):
		return float('inf')

	dataPoint = data.ranges[i]
	if dataPoint < data.range_min or dataPoint > data.range_max:
		return float('inf')
	
	return dataPoint

def callback(data):
	global forward_projection
	print("Data len: ", len(data.ranges))
	print("Angle min: ", data.angle_min)
	print("Angle max: ", data.angle_max)
	print("Increment: ", data.angle_increment)
	print("Range min: ", data.range_min)
	print("Range max: ", data.range_max)
	theta = -25 # you need to try different values for theta
	print(math.radians(theta))

	a = getRange(data, math.radians(theta)) # obtain the ray distance for theta
	front = getRange(data, 0)
	b = getRange(data, math.radians(-90))	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	
	print("Distance side: ", b)
	print("Distance in front: ", front)
	swing = math.radians(theta + 90)

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	# TODO: implement


	alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
	print("alpha: ", math.degrees(alpha))
	ab = b * math.cos(alpha)
	print("Ab: ", ab)
	cd = ab + forward_projection * math.sin(alpha)
	print("Cd: ", cd)
	error = desired_distance - cd
	print("Error: ", error)
	# print("desired distance: ", desired_distance)
	if(math.isnan(error)):
		error = -10

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	if math.isnan(front):
		front = 10
	msg.pid_vel = front		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_4/scan",LaserScan,callback)
	rospy.spin()
