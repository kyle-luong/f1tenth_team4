#!/usr/bin/env python
import math
import rospy
# from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class FTGController:
	def __init__(self):
		self.min_vel = 15.0
		self.max_vel = 25.0
		self.steering_gain = 1
		self.min_gap_threshold = 0.5

		# should this rlly not be /car_4/offboard/command bruh
		# TODO uncomment this is only commented for testing purposes
		self.drive_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 10)
		rospy.Subscriber('/disparity_scan', LaserScan, self.scan_callback)
		self.targetPoint = rospy.Publisher("/sphere_marker", Marker, queue_size = 20)
		self.targets = rospy.Publisher("/target_scan", LaserScan, queue_size = 20)
	

	def scan_callback(self, scan_msg):
		# scan and publish drive cmd
		# not too sure rnscan_msg
		ranges = scan_msg.ranges
		gap_idx, gap_dist, updated_ranges = self.find_gap(ranges, scan_msg)

		# make new LaserScan msg with extended disparities   
		new_scan = LaserScan()
		new_scan.header = scan_msg.header
		new_scan.angle_min = scan_msg.angle_min
		new_scan.angle_max = scan_msg.angle_max
		new_scan.angle_increment = scan_msg.angle_increment
		new_scan.time_increment = scan_msg.time_increment
		new_scan.scan_time = scan_msg.scan_time
		new_scan.range_min = scan_msg.range_min
		new_scan.range_max = scan_msg.range_max
		new_scan.ranges = updated_ranges
		new_scan.intensities = scan_msg.intensities

		arrow_marker = Marker()
		arrow_marker.header.frame_id = "car_4_laser"
		arrow_marker.type = 2
		arrow_marker.header.stamp = rospy.Time.now()
		arrow_marker.id = 1

		angle = gap_idx / float(len(ranges)) * (scan_msg.angle_max - scan_msg.angle_min) + scan_msg.angle_min
		quat = quaternion_from_euler(angle, angle, angle)
		print(math.degrees(angle))
		arrow_marker.pose.position.x = gap_dist
		# Set the scale of the marker
		arrow_marker.scale.x = 0.2
		arrow_marker.scale.y = 0.2
		arrow_marker.scale.z = 0.2
		arrow_marker.pose.orientation.x = quat[0]
		arrow_marker.pose.orientation.y = quat[1]
		arrow_marker.pose.orientation.z = quat[2]
		arrow_marker.pose.orientation.w = quat[3]
		print(gap_idx)

		# Set the colorta
		arrow_marker.color.r = 0.0
		arrow_marker.color.g = 1.0
		arrow_marker.color.b = 0.0
		arrow_marker.color.a = 1.0

		self.targetPoint.publish(arrow_marker)
		self.targets.publish(new_scan)

		steering_angle = self.calculate_steering(ranges, gap_idx, scan_msg)
		# print('steering angle', steering_angle)
		cornering_steering_angle = self.cornering(ranges, steering_angle,scan_msg.angle_min, scan_msg.angle_increment)
		velocity = self.calculate_velocity(ranges, gap_dist)
		self.publish_drive(steering_angle, velocity)

	def find_gap(self, ranges, scan):
		updated_ranges = []
		angleMin = scan.angle_min
		angleMax = scan.angle_max
		gapStart = -90 * math.pi / 180
		gapEnd = 90 * math.pi / 180
		indexStart = int(math.floor((gapStart - angleMin) / (angleMax - angleMin) * (len(ranges))))
		indexEnd = int(math.floor((gapEnd - angleMin) / (angleMax - angleMin) * len(ranges)))

		for i in range(0, indexStart):
			updated_ranges.append(0)
		for i in range(indexStart, indexEnd):
			r = ranges[i]
			if math.isinf(r) or math.isnan(r):
				updated_ranges.append(0.0)
			else:
				updated_ranges.append(r)
		for i in range(indexEnd, len(ranges)):
			updated_ranges.append(0)

		# naive approach
		# curr_run = 0
		# max_run = -1
		# start_index = 0
		# end_index = 0
		# for i, range in enumerate(updated_ranges):
		# 	if range > self.min_gap_threshold:
		# 		curr_run += 1left
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
		return updated_ranges.index(max(updated_ranges)), max(updated_ranges), updated_ranges

		# angle_min = scan.angle_min
		# angle_increment = scan.angle_increment
		# for i in range(len(ranges)):
		# 	best_dist = 0
		# 	best_angle = 0
		# 	angle = angle_min + i * angle_increment
		# 	angle_deg = math.degrees(angle)
		# 	if -90 <= angle_deg <= 90:
		# 		if ranges[i] > best_dist:
		# 			best_dist = ranges[i]
		# 			best_angle = angle_deg
		# return best_angle
	

	def calculate_steering(self, ranges, gap_idx, scan):
		# if the gap_idx is on the right side, then turn right, else turn left
		# for example, [1,3,1,1,3,2,3,4,4,9,3] -> we can clearly see that '9' is on the right side of the lidar scan, so make the car steer to the right

		# TODO get steering angle from target
		size = len(ranges) - 1

		# angle = gap_idx / len(ranges) * (scan.angle_max - scan.angle_min) + scan.angle_min
		# angle *= 180 / math.pi 

		# maybe positive is turning left and negative means turning right
		delta = gap_idx - size/2.0 
		angle = delta/(size/2.0) * 90
		# angle = math.degrees(gap_idx / float(len(ranges)) * (scan.angle_max - scan.angle_min) + scan.angle_min)

		# correcting the direction
		angle *= -1
		angle = max(-100, min(100, angle))

		return angle
	
	# extender we can greedily pick the furthest point
		# return updated_ranges.index(max(updated_ranges)), max(updated_ranges), updated_ranges
			

	def cornering(self, ranges, desired_angle, angle_min, angle_increment):

		distance = 0.2
		min_turn_angle = 10
		left_clear = True
		right_clear = True

		desired_angle_deg = math.degrees(desired_angle)

		for i in range(len(ranges)):
			angle = angle_min + i * angle_increment
			if 90 < angle <= 180:
				if ranges[i] < distance:
					left_clear = False

			if -180 <= angle < -90:
				if ranges[i] < distance:
					right_clear = False

			if desired_angle_deg > min_turn_angle and not left_clear:
				return 0
			if desired_angle_deg  < -min_turn_angle and not right_clear:
				return 0
		
		return desired_angle


	def calculate_velocity(self, ranges, gap_distance):
		# TODO dynamic velocity
		# we can do dynamically based off of the gap distance (?)
		# could also adjust based off of the angle we need to change
		# could also adjust based off of the closest object

		# if the gap is further than 3 meters, drive the max speed
		if gap_distance > 3:
			return self.max_vel

		# if the gap is really close, drive the min velocity
		if gap_distance < 0.5:
			return self.min_vel
		
		# else, scale based on the distance
		velocity = gap_distance/3.0 * self.max_vel
		velocity = max(self.min_vel, min(self.max_vel, velocity))

		return velocity

	def publish_drive(self, steering_angle, velocity):
		# publish drive
		cmd = AckermannDrive()
		cmd.steering_angle = steering_angle
		cmd.speed =  velocity
		self.drive_pub.publish(cmd)


if __name__ == '__main__':

	# rospy.init_node('ftg_controller', anonymous=True)
	# FTGController()
	# rospy.spin()

	ranges = [i for i in range(1080)]

	# # # gap finder test
	ftg = FTGController()
	# ranges = [1,3,5,7,8,4,1,1,1,0,0,1,1,1,3,3,7,8,9,4,6,8,0,1,1,2]
	print(ftg.find_gap(ranges, None))

	# # # velocity test
	# print(ftg.calculate_velocity(ranges, 8))
	# print(ftg.calculate_velocity(ranges, 1))
	# print(ftg.calculate_velocity(ranges, 2))
	# print(ftg.calculate_velocity(ranges, 6))

	# # steering angle test
	# print(ftg.calculate_steering(ranges, 26, None))
	# print(ftg.calculate_steering(ranges, 1, None))
	# print(ftg.calculate_steering(ranges, 12, None))
	# print(ftg.calculate_steering(ranges, 4, None))
