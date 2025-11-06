import disparity_extender
import rospy
from sensor_msgs.msg import LaserScan
import unittest
import math

def test_extend_disparities():
    # Create a sample LaserScan message
    scan = LaserScan()
    scan.angle_min = -math.pi / 2
    scan.angle_max = math.pi / 2
    scan.angle_increment = math.pi / 180  # 1 degree increment
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0] * 90 + [5.0] + [1.0] * 90  # Introduce a disparity at index 90

    threshold = 2.0  # Set disparity threshold
    tolerance = 0.1  # Set tolerance for car width extension

    # Call the extendDisparities function
    extended_scan = disparity_extender.extendDisparities(scan, threshold, tolerance)

    # Check if the disparity has been extended correctly
    for i in range(85, 96):  # Check indices around the disparity
        assert extended_scan.ranges[i] == min(scan.ranges[90], scan.ranges[89], scan.ranges[91]), "Disparity not extended correctly"