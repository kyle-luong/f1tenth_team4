#!/usr/bin/env python
import math
import rospy
import numpy as np
from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class FTGController:
	def __init__(self):
		self.min_vel = 10.0
		self.max_vel = 20.0
		self.steering_gain = 2
		# self.min_gap_threshold = 0.5
		self.angle_min = 0
		self.angle_max = 0
		self.angle_increment = 0


		self.drive_pub = rospy.Publisher('/car_4/multiplexer/command', AckermannDrive, queue_size = 10)
		rospy.Subscriber('/disparity_scan', LaserScan, self.scan_callback)
		self.targetPoint = rospy.Publisher("/sphere_marker", Marker, queue_size = 20)
		self.targets = rospy.Publisher("/target_scan", LaserScan, queue_size = 20)
	
	def preprocess_lidar(self, ranges, scan):
		# Convert to numpy and filter to front view (-90 to +90)
		ranges = np.array(ranges)
		angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
		angles_deg = np.degrees(angles)

		self.angle_increment = scan.angle_increment
		self.angle_min = scan.angle_min
		self.angle_max = scan.angle_max
		
		mask = (angles_deg >= -90) & (angles_deg <= 90)
		front_ranges = ranges.copy()
		
		# Zero out points outside front view
		front_ranges[~mask] = 0.0
		front_ranges[~np.isfinite(front_ranges)] = 0.0
		
		return front_ranges.tolist()

	def scan_callback(self, scan_msg):
		# scan and publish drive cmd
		# not too sure rnscan_msg
		processed_ranges = self.preprocess_lidar(scan_msg.ranges, scan_msg)
		gap_idx, gap_dist = self.find_gap(processed_ranges)

		steering_angle = self.calculate_steering(gap_idx, len(processed_ranges))
		velocity = self.calculate_velocity(gap_dist)
		# cornering_steering_angle = self.cornering(processed_ranges, steering_angle,scan_msg.angle_min, scan_msg.angle_increment)
		self.visualize(scan_msg, processed_ranges, gap_idx, gap_dist)
		self.publish_drive(steering_angle, velocity)

	def find_gap(self, ranges):
		# bc of extender we can greedily pick the furthest point
		if max(ranges) == 0:
			return len(ranges)//2, 0.0
		gap_idx = ranges.index(max(ranges))
		gap_dist = max(ranges)
		return gap_idx, gap_dist
	

	def calculate_steering(self, gap_idx, total_len):
		size = total_len - 1

		# angle = gap_idx / len(ranges) * (scan.angle_max - scan.angle_min) + scan.angle_min
		# angle *= 180 / math.pi 
		# angle = math.degrees(gap_idx / float(len(ranges)) * (scan.angle_max - scan.angle_min) + scan.angle_min)

		# delta = gap_idx - size/2.0 
		# angle = delta/(size/2.0) * 90

		# angle *= -1
		# angle = max(-100, min(100, angle))
		# print('steering angle', angle)

		angle = self.angle_min + gap_idx * self.angle_increment
		print("steering angle:", angle)

		return -1 * self.steering_gain * math.degrees(angle)

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


	def calculate_velocity(self, gap_distance):
		# dynamic velocity
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
		print("velocity", velocity)

		return velocity

	def publish_drive(self, steering_angle, velocity):
		# publish drive
		cmd = AckermannDrive()
		cmd.steering_angle = steering_angle
		cmd.speed =  velocity
		self.drive_pub.publish(cmd)

	def visualize(self, scan_msg, processed_ranges, gap_idx, gap_dist):
		# Publish processed scan
		new_scan = LaserScan()
		new_scan.header = scan_msg.header
		new_scan.angle_min = scan_msg.angle_min
		new_scan.angle_max = scan_msg.angle_max
		new_scan.angle_increment = scan_msg.angle_increment
		new_scan.time_increment = scan_msg.time_increment
		new_scan.scan_time = scan_msg.scan_time
		new_scan.range_min = scan_msg.range_min
		new_scan.range_max = scan_msg.range_max
		new_scan.ranges = processed_ranges
		new_scan.intensities = scan_msg.intensities
		self.targets.publish(new_scan)

		# Publish target marker
		arrow_marker = Marker()
		arrow_marker.header.frame_id = "car_4_laser"
		arrow_marker.type = 2
		arrow_marker.header.stamp = rospy.Time.now()
		arrow_marker.id = 1

		angle = scan_msg.angle_min + gap_idx * scan_msg.angle_increment
		quat = quaternion_from_euler(0, 0, angle)
		arrow_marker.pose.position.x = gap_dist * math.cos(angle)
		arrow_marker.pose.position.y = gap_dist * math.sin(angle)
		arrow_marker.scale.x = 0.2
		arrow_marker.scale.y = 0.2
		arrow_marker.scale.z = 0.2
		arrow_marker.pose.orientation.x = quat[0]
		arrow_marker.pose.orientation.y = quat[1]
		arrow_marker.pose.orientation.z = quat[2]
		arrow_marker.pose.orientation.w = quat[3]
		arrow_marker.color.r = 0.0
		arrow_marker.color.g = 1.0
		arrow_marker.color.b = 0.0
		arrow_marker.color.a = 1.0
		self.targetPoint.publish(arrow_marker)


if __name__ == '__main__':

	rospy.init_node('ftg_controller', anonymous=True)
	FTGController()
	rospy.spin()
