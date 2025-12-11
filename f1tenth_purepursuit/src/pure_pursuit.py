#!/usr/bin/env python

import os
import sys
import csv
import math
import numpy as np
import rospy
import tf
import rvizrace

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

plan = []
path_resolution = []

frame_id = "map"
car_name = "car_4"
trajectory_name = "test_raceline"

command_pub = rospy.Publisher("/%s/offboard/command" % car_name, AckermannDrive, queue_size=100)
polygon_pub = rospy.Publisher("/%s/purepursuit_control/visualize" % car_name, PolygonStamped, queue_size=1)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
path_pub = rospy.Publisher("/%s/purepursuit/path" % car_name, Path, queue_size=1)
arrow_marker_pub = rospy.Publisher("/arrow_marker", Marker, queue_size = 20)

wp_seq = 0
control_polygon = PolygonStamped()

STEERING_RANGE = 100.0
WHEELBASE_LEN = 0.325


def _get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def construct_path():
    path_file = os.path.expanduser(
        "~/catkin_ws/src/f1tenth_purepursuit/racelines/%s.csv" % trajectory_name
    )

    if not os.path.exists(path_file):
        rospy.logerr("Path file not found: %s" % path_file)
        return

    with open(path_file) as f:
        reader = csv.reader(f, delimiter=",")
        for wp in reader:
            plan.append([float(wp[0]), float(wp[1])])

    for i in range(1, len(plan)):
        dx = plan[i][0] - plan[i - 1][0]
        dy = plan[i][1] - plan[i - 1][1]
        path_resolution.append(math.sqrt(dx * dx + dy * dy))

    rospy.loginfo("Loaded %d waypoints" % len(plan))


def publish_path():
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()

    for p in plan:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

    path_pub.publish(msg)


def get_param_or_input(name, default, cast=float):
    param_name = "~" + name

    if rospy.has_param(param_name):
        try:
            return cast(rospy.get_param(param_name))
        except Exception as e:
            rospy.logwarn("Param %s cast failed: %s. Using default." % (param_name, e))
            return default

    try:
        v = input("%s [%s]: " % (name, default)).strip()
        return cast(v) if v else default
    except Exception:
        return default


# def local_curvature(i):
    i0 = max(0, i - 1)
    i1 = i
    i2 = min(len(plan) - 1, i + 1)

    p0 = np.array(plan[i0])
    p1 = np.array(plan[i1])
    p2 = np.array(plan[i2])

    a = np.linalg.norm(p1 - p0)
    b = np.linalg.norm(p2 - p1)
    c = np.linalg.norm(p2 - p0)

    s = 0.5 * (a + b + c)
    area = math.sqrt(max(0.0, s * (s - a) * (s - b) * (s - c)))

    if area < 1e-9 or a < 1e-9 or b < 1e-9 or c < 1e-9:
        return 0.0

    R = (a * b * c) / (4.0 * area)

    if math.isinf(R) or math.isnan(R) or R < 1e-6:
        return 0.0

    return 1.0 / R


def purepursuit_control_node(data):
    global wp_seq

    if not plan:
        return

    command = AckermannDrive()

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    odom = [odom_x, odom_y]

    # find closest point on path
    def get_closest_index():
        min_dist = float("inf")
        min_idx = 0
        for idx, pt in enumerate(plan):
            d = _get_distance(odom, pt)
            if d < min_dist:
                min_dist = d
                min_idx = idx
        return min_idx

    min_idx = get_closest_index()

    # robot heading
    orientation_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )
    heading = tf.transformations.euler_from_quaternion(orientation_quat)[2]

    # k = local_curvature(min_idx)

    max_vel = get_param_or_input("max_vel", 60.0, float)
    min_vel = get_param_or_input("min_vel", 30.0, float)

    # speed based on steering
    def calculate_velocity(angle):
        a = abs(angle)
        angle_max = 40.0
        if a < 7.77:
            return max_vel
        if a > angle_max:
            return min_vel
        return (angle_max - a) / angle_max * (max_vel - min_vel) + min_vel

    # lookahead based on speed
    def adaptive_lookahead(v):
        if (max_vel - min_vel) > 0:
            p = (v - min_vel) / (max_vel - min_vel)
        else:
            p = 0.0
        min_lookahead = 0.2
        max_lookahead = 3
        lookahead = min_lookahead + (max_lookahead - min_lookahead) * p

        return lookahead

    velocity = calculate_velocity(command.steering_angle)
    lookahead_distance = adaptive_lookahead(velocity)

    # pick lookahead target point
    def get_target_point():
        idx = min_idx
        while idx < len(plan) - 1:
            if _get_distance(odom, plan[idx]) >= lookahead_distance:
                break
            idx += 1
        return plan[max(0, idx - 1)]

    target_pt = get_target_point()

    # steering calculation
    def calculate_steering():
        tx = target_pt[0] - odom[0]
        ty = target_pt[1] - odom[1]

        target_vec = [tx, ty, 0, 0]

        conj = tf.transformations.quaternion_conjugate(orientation_quat)
        v1 = tf.transformations.quaternion_multiply(conj, target_vec)
        car_frame = tf.transformations.quaternion_multiply(v1, orientation_quat)

        alpha = math.asin(car_frame[1] / lookahead_distance)
        steer = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / lookahead_distance)
        angle = math.degrees(steer)
        
        return angle

    s_angle = calculate_steering()
    angle = 5 * calculate_steering()
    angle = max(-STEERING_RANGE, min(STEERING_RANGE, angle))

    command.steering_angle = angle

    velocity = calculate_velocity(command.steering_angle)
    lookahead_distance = adaptive_lookahead(velocity)

    command.speed = velocity


    rospy.loginfo_throttle(0.1,
        "ld=%.2f v=%.2f angle=%.2f" %
        (lookahead_distance, command.speed, command.steering_angle)
    )

    command_pub.publish(command)

    # visualization points
    base = Point32(x=odom_x, y=odom_y)
    pose_pt = Point32(x=odom_x, y=odom_y)
    goal_pt = Point32(x=target_pt[0], y=target_pt[1])

    control_polygon.header.frame_id = frame_id
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    control_polygon.polygon.points = [pose_pt, base, goal_pt]

    wp_seq += 1

    polygon_pub.publish(control_polygon)
    raceline_pub.publish(rvizrace.raceline_path)

    # ARROW MARKER

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    arrow_marker = Marker()
    arrow_marker.header.frame_id = "map"
    arrow_marker.type = 0
    arrow_marker.header.stamp = rospy.Time.now()
    arrow_marker.id = 1
    arrow_marker.scale.x = 0.6 # data.ranges[540]
    arrow_marker.scale.y = 0.1
    arrow_marker.scale.z = 0.1

    arrow_marker.pose.position.x = odom_x
    arrow_marker.pose.position.y = odom_y
    arrow_marker.pose.position.z = 0.1
    
    quat = quaternion_from_euler(0, 0, math.radians(angle))
    arrow_marker.pose.orientation.x = quat[0]
    arrow_marker.pose.orientation.y = quat[1]
    arrow_marker.pose.orientation.z = quat[2]
    arrow_marker.pose.orientation.w = quat[3]

    # Set the colorta
    arrow_marker.color.r = 0.0
    arrow_marker.color.g = 0.0
    arrow_marker.color.b = 1.0
    arrow_marker.color.a = 1.0

    arrow_marker_pub.publish(arrow_marker)


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)

        if not plan:
            rospy.loginfo("Loading trajectory...")
            construct_path()

            if plan:
                publish_path()
                rospy.loginfo("Trajectory loaded.")
            else:
                rospy.logerr("No trajectory loaded. Check CSV path.")

        rospy.Subscriber(
            "/%s/particle_filter/viz/inferred_pose" % car_name,
            PoseStamped,
            purepursuit_control_node
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass