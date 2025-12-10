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



plan = []
path_resolution = []

frame_id = "map"
car_name = "car_4"
trajectory_name = "test_raceline"

command_pub = rospy.Publisher(f"/{car_name}/offboard/command", AckermannDrive, queue_size=100)
polygon_pub = rospy.Publisher(f"/{car_name}/purepursuit_control/visualize", PolygonStamped, queue_size=1)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
path_pub = rospy.Publisher(f"/{car_name}/purepursuit/path", Path, queue_size=1)

wp_seq = 0
control_polygon = PolygonStamped()

STEERING_RANGE = 100.0
WHEELBASE_LEN = 0.325


def _get_distance(p1, p2):
    """Euclidean distance between two points."""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def construct_path():
    """Load waypoints from CSV into 'plan'."""
    file_path = os.path.expanduser(
        f"~/catkin_ws/src/f1tenth_purepursuit/racelines/{trajectory_name}.csv"
    )

    if not os.path.exists(file_path):
        rospy.logerr(f"Path file not found: {file_path}")
        return

    with open(file_path) as csv_file:
        reader = csv.reader(csv_file, delimiter=",")
        for waypoint in reader:
            plan.append([float(waypoint[0]), float(waypoint[1])])

    for i in range(1, len(plan)):
        dx = plan[i][0] - plan[i - 1][0]
        dy = plan[i][1] - plan[i - 1][1]
        path_resolution.append(math.sqrt(dx * dx + dy * dy))

    rospy.loginfo(f"Loaded {len(plan)} waypoints.")


def publish_path():
    """Publish the loaded path to RViz."""
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
    """Get ROS param or fallback to user input."""
    param_name = "~" + name

    if rospy.has_param(param_name):
        try:
            return cast(rospy.get_param(param_name))
        except Exception as e:
            rospy.logwarn(f"Param {param_name} cast failed: {e}. Using default.")
            return default

    try:
        v = input(f"{name} [{default}]: ").strip()
        return cast(v) if v else default
    except Exception:
        return default


def local_curvature(i):
    """Return curvature computed from three consecutive points."""
    i0 = max(0, i - 1)
    i1 = i
    i2 = min(len(plan) - 1, i + 1)

    p0, p1, p2 = np.array(plan[i0]), np.array(plan[i1]), np.array(plan[i2])

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
    """Perform pure pursuit control given the current pose."""
    global wp_seq

    if not plan:
        return

    command = AckermannDrive()

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    odom = [odom_x, odom_y]

    def get_base_projection():
        min_dist = float("inf")
        min_idx = 0
        for idx, pt in enumerate(plan):
            d = _get_distance(odom, pt)
            if d < min_dist:
                min_dist = d
                min_idx = idx
        return min_idx

    min_idx = get_base_projection()

    orientation_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )

    heading = tf.transformations.euler_from_quaternion(orientation_quat)[2]

    k = local_curvature(min_idx)

    max_vel = get_param_or_input("max_vel", 60.0, float)
    min_vel = get_param_or_input("min_vel", 33.5, float)

    def calculate_velocity(angle):
        a = abs(angle)
        angle_max = 40.0
        if a > angle_max:
            return min_vel
        return (angle_max - a) / angle_max * (max_vel - min_vel) + min_vel

    def adaptive_lookahead(v):
        p = (v - min_vel) / (max_vel - min_vel) if (max_vel - min_vel) > 0 else 0.0
        l_min, l_max = 0.75, 2.0
        return (l_max - l_min) * p + l_min

    velocity = calculate_velocity(command.steering_angle)
    lookahead_distance = adaptive_lookahead(velocity)

    def get_target_point():
        idx = min_idx
        while idx < len(plan) - 1:
            if _get_distance(odom, plan[idx]) >= lookahead_distance:
                break
            idx += 1

        return plan[max(0, idx - 1)]

    target_pt = get_target_point()


    def calculate_steering():
        tx = target_pt[0] - odom[0]
        ty = target_pt[1] - odom[1]

        target_vec = [tx, ty, 0, 0]

        conj = tf.transformations.quaternion_conjugate(orientation_quat)
        v1 = tf.transformations.quaternion_multiply(conj, target_vec)
        car_frame_target = tf.transformations.quaternion_multiply(v1, orientation_quat)

        alpha = math.asin(car_frame_target[1] / lookahead_distance)
        steer = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / lookahead_distance)

        return math.degrees(steer)

    angle = 5 * calculate_steering()
    angle = max(-STEERING_RANGE, min(STEERING_RANGE, angle))

    command.steering_angle = angle
    command.speed = velocity

    rospy.loginfo_throttle(
        1.0,
        f"ld={lookahead_distance:.2f} v={command.speed:.2f} "
        f"angle={command.steering_angle:.2f} k={k:.3f}",
    )

    command_pub.publish(command)

   
    base = Point32(x=odom_x, y=odom_y)
    nearest_pose = Point32(x=odom_x, y=odom_y)
    nearest_goal = Point32(x=target_pt[0], y=target_pt[1])

    control_polygon.header.frame_id = frame_id
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    control_polygon.polygon.points = [nearest_pose, base, nearest_goal]

    wp_seq += 1
    polygon_pub.publish(control_polygon)
    raceline_pub.publish(rvizrace.raceline_path)


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
            f"/{car_name}/particle_filter/viz/inferred_pose",
            PoseStamped,
            purepursuit_control_node,
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass