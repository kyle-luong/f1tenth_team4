#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

car_length = 0.50 # meters
car_width = 0.26 # meters

pub = rospy.Publisher('disparity_scan', LaserScan, queue_size=10)


def extendDisparities(data, threshold, tolerance):
	# data: single message from topic /scan
	# threshold: disparity threshold
    # tolerance: value in meters to extend car width on either side

    start = data.angle_min
    increment = data.angle_increment
    range_min = data.range_min
    range_max = data.range_max

    # compute clearance distance
    clearance = (car_width / 2) + tolerance

    # copy lidar ranges
    data_copy = list(data.ranges)

    # compare lidar scan at index i to the one next to it at i + 1 to find disparity
    for i in range(len(data.ranges) - 1):
        r1 = data.ranges[i]
        r2 = data.ranges[i + 1]

        # filter out bad lidar readings, move to next i if met
        if (r1 < range_min or r1 > range_max or r2 < range_min or r2 > range_max):    
            continue

        disparity = abs(r1 - r2)

        if disparity > threshold:
            # compute the angle of the disparity 
            angle = math.degrees(start + i * increment)

            # only consider -90 to 90, not behind the car
            if -90 <= angle <= 90:
                
                # determine direction to step through the lidar scan to overwrite values
                if r1 <= r2:
                    d = r1
                    step = 1
                    start_index = i + 1
                else:
                    d = r2
                    step = -1
                    start_index = i
            
                # figure out number of samples needed to extend disparity
                theta = math.atan2(clearance, d)
                number_samples = int(math.ceil(theta / increment))

                j = start_index
                c = 0

                # iterate through a laserscan and extend disparities
                while c < number_samples:
                    if j < 0:
                        break
                    if j >= len(data_copy):
                        break
                    
                    # overwrite range 
                    data_copy[j] = d

                    j += step
                    c += 1

    # make new LaserScan msg with extended disparities   
    new_scan = LaserScan()
    new_scan.header = data.header
    new_scan.angle_min = data.angle_min
    new_scan.angle_max = data.angle_max
    new_scan.angle_increment = data.angle_increment
    new_scan.time_increment = data.time_increment
    new_scan.scan_time = data.scan_time
    new_scan.range_min = data.range_min
    new_scan.range_max = data.range_max
    new_scan.ranges = data_copy
    new_scan.intensities = data.intensities

    return new_scan


def callback(data):
    disparity_scan = extendDisparities(data, 0.3, 0.025)
    pub.publish(disparity_scan)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparity_extender',  anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_4/scan", LaserScan, callback)
	rospy.spin()
