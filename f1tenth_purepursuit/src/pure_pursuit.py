#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import csv
import math
import numpy as np
import tf
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path
import rvizrace

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = "map"
# car_name            = str(sys.argv[1])
# trajectory_name     = str(sys.argv[2])
car_name            = "car_4"
trajectory_name     = "test_raceline"

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher("/{}/offboard/command".format(car_name), AckermannDrive, queue_size=100)
polygon_pub         = rospy.Publisher("/{}/purepursuit_control/visualize".format(car_name), PolygonStamped, queue_size=1)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)

# visualize refrence path
path_pub = rospy.Publisher("/{}/purepursuit/path".format(car_name), Path, queue_size=1)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq          = 0
control_polygon = PolygonStamped()

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN = 0.325

def dist(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser("~/catkin_ws/src/f1tenth_purepursuit/racelines/{}.csv".format(trajectory_name))
    if not os.path.exists(file_path):
        rospy.logerr("Path file not found: {}".format(file_path))
        return
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ",")
        for waypoint in csv_reader:
            plan.append([float(waypoint[0]), float(waypoint[1])])
    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))
    rospy.loginfo("Loaded {} waypoints".format(len(plan)))


def publish_path():
    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    for p in plan:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    path_pub.publish(msg)

def nearest_index(position):
    best_i = 0
    best_d = float("inf")
    for i, p in enumerate(plan):
        d = dist(position, p)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i

def target_point(position, start_i, lookahead):
    i = start_i
    if i >= len(plan):
        return plan[-1][0], plan[-1][1]
    while i < len(plan) and dist(position, plan[i]) < lookahead:
        i += 1
    i = max(0, min(len(plan) - 1, i))
    return plan[i][0], plan[i][1]

def car_frame_target(odom, target, orientation_quat):
    dx = target[0] - odom[0]
    dy = target[1] - odom[1]
    q = orientation_quat
    conj = tf.transformations.quaternion_conjugate(q)
    v = [dx, dy, 0.0, 0.0]
    v = tf.transformations.quaternion_multiply(conj, v)
    v = tf.transformations.quaternion_multiply(v, q)
    return v[0], v[1]

def steering_angle_deg(y_rel, x_rel, lookahead):
    alpha = math.atan2(y_rel, max(1e-6, x_rel))
    delta = math.atan2(2.0 * WHEELBASE_LEN * math.sin(alpha), max(1e-6, lookahead))
    return math.degrees(delta)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def calculate_velocity(angle_deg, min_vel, max_vel):
    a = abs(angle_deg)
    angle_max = 40.0
    if a > angle_max:
        return min_vel
    t = (angle_max - a) / angle_max
    return t * (max_vel - min_vel) + min_vel

def adaptive_lookahead(v, min_vel, max_vel):
    if max_vel <= min_vel:
        return 1.0
    p = (v - min_vel) / (max_vel - min_vel)
    p = clamp(p, 0.0, 1.0)
    l_min = 0.75
    l_max = 2.0
    return (l_max - l_min) * p + l_min

def purepursuit_control_node(data):
    global wp_seq

    if not plan:
        return

    cmd = AckermannDrive()

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    odom = [odom_x, odom_y]

    orientation = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )

    heading = tf.transformations.euler_from_quaternion(orientation)[2]

    max_vel = rospy.get_param("~max_vel", 60.0)
    min_vel = rospy.get_param("~min_vel", 33.5)

    i_near = nearest_index(odom)
    target_x, target_y = target_point(odom, i_near, adaptive_lookahead(max_vel, min_vel, max_vel))

    x_rel, y_rel = car_frame_target(odom, (target_x, target_y), orientation)
    angle_deg = 5.0 * steering_angle_deg(y_rel, x_rel, adaptive_lookahead(max_vel, min_vel, max_vel))
    angle_deg = clamp(angle_deg, -STEERING_RANGE, STEERING_RANGE)
    cmd.steering_angle = angle_deg

    velocity = calculate_velocity(angle_deg, min_vel, max_vel)
    cmd.speed = velocity
    command_pub.publish(cmd)

    base_link = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x = odom_x
    base_link.y = odom_y
    nearest_pose.x = odom_x
    nearest_pose.y = odom_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y

    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    wp_seq += 1
    polygon_pub.publish(control_polygon)
    raceline_pub.publish(rvizrace.raceline_path)

def main():
    rospy.init_node("pure_pursuit", anonymous=True)
    rospy.loginfo("pure_pursuit node starting")
    construct_path()
    if plan:
        publish_path()
        rospy.loginfo("trajectory published")
    else:
        rospy.logerr("no trajectory loaded; node will still run but do nothing until path is available")
    rospy.Subscriber("/{}/particle_filter/viz/inferred_pose".format(car_name), PoseStamped, purepursuit_control_node)
    rospy.loginfo("subscribed to inferred_pose")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass