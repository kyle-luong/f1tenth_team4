#!/usr/bin/env python

# raceline pub definition
import rospy
from nav_msgs.msg import Path
import csv
import os
import time
from geometry_msgs.msg import PoseStamped


# raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)

# read in raceline as usual
plan = []

file_path = os.path.expanduser(
    "~/catkin_ws/src/f1tenth_purepursuit/racelines/test_raceline.csv"
)
file = open(file_path, 'r')
csv_reader = csv.reader(file, delimiter=",")
for waypoint in csv_reader:
    plan.append(waypoint)

# create path object

raceline_path = Path()
raceline_path.header.frame_id = "map"

for index, point in enumerate(plan):
    waypoint = PoseStamped()
    waypoint.header.frame_id = "map"
    # print(point[0])

    waypoint.pose.position.x = float(point[0])
    waypoint.pose.position.y = float(point[1])
    raceline_path.poses.append(waypoint)

  
   
# # raceline_pub.init_node()
# if __name__ == "__main__":
#     rospy.init_node("rvizrace")
#     time.sleep(1)
#     i = 0
#     while i < 10:
# raceline_pub.publish(raceline_path)
# i += 1
# time.sleep(1)
