#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32

# PID Control Params
kp = 100 #TODO
kd = 100 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0

max_error = 1.5 # meters

# dynamic velocity constants
vel_kp = 50
vel_min = 25
vel_max = 45

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 35.0	#TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 10)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	global max_error

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering

	error_msg = data.pid_error

	# scale the error - 0.5 to be normalized -1 to 1
	error_raw = max(-max_error, min(max_error, error_msg))
	error = error_raw / max_error

	p = kp * error
	d = kd * (error - prev_error)

	angle = p + d

	# clamp steering angle -100 to 100s
	if angle > 100:
		angle = 100
	elif angle < -100:
		angle = -100

	angle = -angle

	# dynamic velocity
	vel_error = abs(data.pid_error)

	if vel_error > max_error:
		vel_error = max_error
	
	vel_input = vel_max - (vel_max - vel_min) * (vel_error)
	print('vel error: ',  vel_error)

	# clamp velocity from vel_min to vel_max
	if vel_input > vel_max:
		vel_input = vel_max
	elif vel_input < vel_min:
		vel_input = vel_min

	# TODO: Make sure the steering value is within bounds [-100,100]
	# TODO: Make sure the velocity is within bounds [0,100]

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
	command.steering_angle = angle
	command.speed = vel_input	


	# Move the car autonomously
	command_pub.publish(command)

	prev_error = error

	print("Steering angle is ", angle)
	print("Velocity is ", vel_input)
	print("Error is ", error_raw)
	print("velocity error is ", vel_error)

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
