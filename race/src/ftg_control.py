#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32


servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0


# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 10)

def control(data):
	global prev_error
	global vel_max
	global kp
	global kd
	global angle
	global max_error

	
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
	command.steering_angle = 0
	command.speed = 0	


	# Move the car autonomously
	command_pub.publish(command)


if __name__ == '__main__':

	rospy.init_node('ftg_controller', anonymous=True)
	rospy.spin()
