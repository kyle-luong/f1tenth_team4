#!/usr/bin/env python
import math
import rospy
# from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32

class FTGController:
	def __init__(self):
		self.min_vel = 15.0
		self.max_vel = 15.0
		self.steering_gain = 1
		self.min_gap_threshold = 0.5

		# should this rlly not be /car_4/offboard/command bruh
		# TODO uncomment this is only commented for testing purposes
		self.drive_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 10)
		rospy.Subscriber('/disparity_scan', LaserScan, self.scan_callback)

	def scan_callback(self, scan_msg):
		# scan and publish drive cmd
		# not too sure rn
		ranges = scan_msg.ranges
		gap_idx, gap_dist = self.find_gap(ranges)
		steering_angle = self.calculate_steering(ranges, gap_idx)
		velocity = self.calculate_velocity(ranges, gap_dist)
		self.publish_drive(steering_angle, velocity)

	def find_gap(self, ranges):
		updated_ranges = []
		for range in ranges:
			if math.isinf(range) or math.isnan(range):
				updated_ranges.append(0.0)
			else:
				updated_ranges.append(range)

		# naive approach
		# curr_run = 0
		# max_run = -1
		# start_index = 0
		# end_index = 0
		# for i, range in enumerate(updated_ranges):
		# 	if range > self.min_gap_threshold:
		# 		curr_run += 1
		# 	else:
		# 		if curr_run > max_run:
		# 			start_index = i - curr_run
		# 			end_index = i - 1
		# 			max_run = curr_run
		# 		curr_run = 0

		# gap_index = int((start_index + end_index) / 2)
		# gap_distance = ranges[gap_index]
		# return gap_index, gap_distance

		# since we have disparity extender we can greedily pick the furthest point
		return updated_ranges.index(max(updated_ranges)), max(updated_ranges)

	def calculate_steering(self, ranges, gap_idx):
		# TODO get steering angle from target
		size = len(ranges)

		#assume positive is turning right and negative means turning left
		delta = gap_idx - size/2.0 
		angle = delta/(size/2.0) * 120

		angle = max(-100, min(100, angle))

		return angle

	def calculate_velocity(self, ranges, gap_distance):
		# TODO dynamic velocity
		# we can do dynamically based off of the gap distance (?)
		# could also adjust based off of the angle we need to change
		# could also adjust based off of the closest object
		velocity = gap_distance/8.0 * self.max_vel
		velocity = max(self.min_vel, min(self.max_vel, velocity))

		return velocity

	def publish_drive(self, steering_angle, velocity):
		# publish drive
		cmd = AckermannDrive()
		cmd.steering_angle = steering_angle
		cmd.speed =  velocity
		self.drive_pub.publish(cmd)


if __name__ == '__main__':

	rospy.init_node('ftg_controller', anonymous=True)
	FTGController()
	rospy.spin()


	# # gap finder test
	# ftg = FTGController()
	# ranges = [1,3,5,7,8,4,1,1,1,0,0,1,1,1,3,3,7,8,9,4,6,8,0,1,1,2]
	# print(ftg.find_gap(ranges))

	# # velocity test
	# print(ftg.calculate_velocity(ranges, 8))
	# print(ftg.calculate_velocity(ranges, 1))
	# print(ftg.calculate_velocity(ranges, 2))
	# print(ftg.calculate_velocity(ranges, 6))

	# # steering angle test
	# print(ftg.calculate_steering(ranges, 20))
	# print(ftg.calculate_steering(ranges, 1))
	# print(ftg.calculate_steering(ranges, 12))
	# print(ftg.calculate_steering(ranges, 4))