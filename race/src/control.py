#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 5 #TODO
kd = 5 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 15.0	#TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 10)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering

	error = data.pid_error
	p = kp * error
	d = kd * (prev_error - error)

	angle = p + d

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	if not (abs(angle) <= 100):
		print("Out of bounds steering angle", angle, " correcting within bounds")
		angle = max(-100, min(100, angle))
	angle = -angle
	command.steering_angle = angle
	print("Steering angle is ", angle)

	# TODO: Make sure the velocity is within bounds [0,100]
	if not (0 <= vel_input <= 100):
		print("Out of bounds velocity input", vel_input, "correcting within bounds")
		vel_input = max(0, min(100, vel_input))
	command.speed = vel_input
	print("Velocity is ", vel_input)

	# Move the car autonomously
	command_pub.publish(command)

	prev_error = error

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	# kp = input("Enter Kp Value: ")
	# kd = input("Enter Kd Value: ")
	# ki = input("Enter Ki Value: ")
	# vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
