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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

plan = []
path_resolution = []

plan2 = []
path_resolution2 = []

frame_id = "map"
car_name = "car_4"
current_scan = None
trajectory_name = "working_raceline"
path_publish_times = 0
trajectory_name2 = "working_raceline"
mode = "primary"  # "primary", "secondary"

command_pub = rospy.Publisher("/%s/offboard/command" % car_name, AckermannDrive, queue_size=100)
polygon_pub = rospy.Publisher("/%s/purepursuit_control/visualize" % car_name, PolygonStamped, queue_size=1)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)  # existing (rvizrace)
path_pub = rospy.Publisher("/%s/purepursuit/path" % car_name, Path, queue_size=1)
arrow_marker_pub = rospy.Publisher("/arrow_marker", Marker, queue_size=20)

# publishers for secondary raceline as Path
raceline2_pub = rospy.Publisher("/raceline2", Path, queue_size=1)

wp_seq = 0
control_polygon = PolygonStamped()

STEERING_RANGE = 100.0
WHEELBASE_LEN = 0.325

# Fixed distance (meters) ahead along raceline for obstacle detection
OBSTACLE_DETECTION_DISTANCE = 1.5

# Minimum time between raceline switches (seconds)
SWITCH_MIN_INTERVAL = 0.6
last_switch_time = 0.0


def _get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def construct_path(trajectory_name):
    path_file = os.path.expanduser(
        "~/catkin_ws/src/f1tenth_purepursuit/racelines/%s.csv" % trajectory_name
    )

    rospy.loginfo("Loading %s waypoints" % trajectory_name)

    if not os.path.exists(path_file):
        rospy.logerr("Path file not found: %s" % path_file)
        return

    this_plan = []

    with open(path_file) as f:
        reader = csv.reader(f, delimiter=",")
        for wp in reader:
            this_plan.append([float(wp[0]), float(wp[1])])

    this_path_resolution = []
    for i in range(1, len(this_plan)):
        dx = this_plan[i][0] - this_plan[i - 1][0]
        dy = this_plan[i][1] - this_plan[i - 1][1]
        this_path_resolution.append(math.sqrt(dx * dx + dy * dy))

    rospy.loginfo("Loaded %d waypoints" % len(this_plan))
    return this_plan, this_path_resolution


def is_path_blocked(target_pt, odom, heading, scan, safety_margin=0.3, window_size=5):
    """
    Returns True if LiDAR indicates an obstacle between the robot and the target point.
    - target_pt: [x, y] in map frame
    - odom: [x, y] in map frame
    - heading: yaw of robot in map frame (rad)
    - scan: LaserScan
    """
    if scan is None:
        return False

    # Vector from robot to target in map frame
    dx = target_pt[0] - odom[0]
    dy = target_pt[1] - odom[1]

    # Transform into robot frame (x forward, y left)
    cos_h = math.cos(-heading)
    sin_h = math.sin(-heading)
    x_r = cos_h * dx - sin_h * dy
    y_r = sin_h * dx + cos_h * dy

    distance = math.hypot(x_r, y_r)
    if distance < 1e-3:
        return False  # Target basically on top of us

    # If target is behind us, treat as blocked / invalid
    if x_r <= 0.0:
        return True

    desired_angle = math.atan2(y_r, x_r)

    idx_center = int((desired_angle - scan.angle_min) / scan.angle_increment)
    idx_center = max(0, min(idx_center, len(scan.ranges) - 1))

    i0 = max(0, idx_center - window_size)
    i1 = min(len(scan.ranges) - 1, idx_center + window_size)

    # Filter out NaNs/Infs
    window_ranges = []
    for r in scan.ranges[i0:i1 + 1]:
        if not math.isinf(r) and not math.isnan(r):
            window_ranges.append(r)

    if not window_ranges:
        return False

    min_range = min(window_ranges)

    return min_range < (distance - safety_margin)


def publish_path():
    # Path used by pure pursuit (primary raceline)
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

    # Original pure pursuit path topic
    path_pub.publish(msg)

    # publish secondary raceline on /raceline2 (if it exists)
    if plan2:
        msg2 = Path()
        msg2.header.frame_id = frame_id
        msg2.header.stamp = rospy.Time.now()
        for p in plan2:
            pose2 = PoseStamped()
            pose2.header.frame_id = frame_id
            pose2.pose.position.x = p[0]
            pose2.pose.position.y = p[1]
            pose2.pose.orientation.w = 1.0
            msg2.poses.append(pose2)
        raceline2_pub.publish(msg2)
    else:
        print "No"


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


def local_curvature(i):
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


def laser_callback(scan_msg):
    """Store latest laser scan"""
    global current_scan
    current_scan = scan_msg


def preprocess_lidar(ranges):
    """Preprocess LiDAR data - remove invalid readings"""
    proc_ranges = []
    for r in ranges:
        if np.isinf(r) or np.isnan(r):
            proc_ranges.append(0.0)
        else:
            proc_ranges.append(r)
    return proc_ranges


def purepursuit_control_node(data):
    global wp_seq, current_scan, plan, plan2, path_publish_times, mode, last_switch_time

    if not plan:
        return

    command = AckermannDrive()

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    odom = [odom_x, odom_y]

    # find closest point on path for a given plan
    def get_closest_index(path):
        min_dist = float("inf")
        min_idx = 0
        for idx, pt in enumerate(path):
            d = _get_distance(odom, pt)
            if d < min_dist:
                min_dist = d
                min_idx = idx
        return min_idx

    min_idx1 = get_closest_index(plan)
    min_idx2 = get_closest_index(plan2) if plan2 else None

    # robot heading
    orientation_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )
    heading = tf.transformations.euler_from_quaternion(orientation_quat)[2]

    max_vel = get_param_or_input("max_vel", 60.0, float)
    min_vel = get_param_or_input("min_vel", 30.0, float)

    # speed based on steering
    def calculate_velocity(angle_deg):
        a = abs(angle_deg)
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
        min_lookahead = 0.5
        max_lookahead = 1.8
        lookahead = min_lookahead + (max_lookahead - min_lookahead) * p
        return lookahead

    v_nominal = max_vel
    lookahead_guess = adaptive_lookahead(v_nominal)

    # pick lookahead target point for a specific plan (pure pursuit)
    def get_target_point(path, start_idx, lookahead_distance):
        idx = start_idx
        while idx < len(path) - 1:
            if _get_distance(odom, path[idx]) >= lookahead_distance:
                break
            idx += 1
        return path[max(0, idx - 1)]

    # fixed-distance detection point along a plan (for obstacle check only)
    def get_detection_point(path, start_idx, detection_distance):
        idx = start_idx
        while idx < len(path) - 1:
            if _get_distance(odom, path[idx]) >= detection_distance:
                break
            idx += 1
        return path[max(0, idx - 1)]

    # steering calculation for a given target
    def calculate_steering_for_target(target_pt, lookahead_distance):
        tx = target_pt[0] - odom[0]
        ty = target_pt[1] - odom[1]

        target_vec = [tx, ty, 0.0, 0.0]

        conj = tf.transformations.quaternion_conjugate(orientation_quat)
        v1 = tf.transformations.quaternion_multiply(conj, target_vec)
        car_frame = tf.transformations.quaternion_multiply(v1, orientation_quat)

        # y-component of target in car frame
        y_c = car_frame[1]
        # avoid division by zero
        if lookahead_distance < 1e-3:
            return 0.0

        alpha = math.asin(max(-1.0, min(1.0, y_c / lookahead_distance)))
        steer = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / lookahead_distance)
        angle_deg = math.degrees(steer)
        return angle_deg

    # PRIMARY PLAN (pure pursuit target)
    target1 = get_target_point(plan, min_idx1, lookahead_guess)
    angle1 = calculate_steering_for_target(target1, lookahead_guess)
    v1 = calculate_velocity(angle1)
    ld1 = adaptive_lookahead(v1)

    # fixed-distance detection point for primary plan
    detection_target1 = get_detection_point(plan, min_idx1, OBSTACLE_DETECTION_DISTANCE)

    # blocked if obstacle near fixed detection OR anywhere between robot and target1
    blocked1_fixed = is_path_blocked(detection_target1, odom, heading, current_scan)
    blocked1_segment = is_path_blocked(target1, odom, heading, current_scan)
    blocked1 = blocked1_fixed or blocked1_segment

    # SECONDARY PLAN
    target2 = None
    angle2 = None
    v2 = None
    ld2 = None
    blocked2 = True

    if plan2 and min_idx2 is not None:
        target2 = get_target_point(plan2, min_idx2, lookahead_guess)
        angle2 = calculate_steering_for_target(target2, lookahead_guess)
        v2 = calculate_velocity(angle2)
        ld2 = adaptive_lookahead(v2)

        # fixed-distance detection point for secondary plan
        detection_target2 = get_detection_point(plan2, min_idx2, OBSTACLE_DETECTION_DISTANCE)

        # blocked if obstacle near fixed detection OR anywhere between robot and target2
        blocked2_fixed = is_path_blocked(detection_target2, odom, heading, current_scan)
        blocked2_segment = is_path_blocked(target2, odom, heading, current_scan)
        blocked2 = blocked2_fixed or blocked2_segment

    # Decide what the controller WOULD like to use this cycle
    preferred_mode = "primary"
    if blocked1 and target2 is not None:
        preferred_mode = "secondary"

    # Enforce minimum time between switches
    now = rospy.Time.now().to_sec()
    if preferred_mode != mode:
        if (now - last_switch_time) >= SWITCH_MIN_INTERVAL:
            mode = preferred_mode
            last_switch_time = now

    # Now select based on the (possibly unchanged) mode
    if mode == "secondary" and target2 is not None:
        selected_target = target2
        selected_angle = angle2
        selected_velocity = v2
        selected_ld = ld2
        used_plan = "secondary"
    else:
        # fall back to primary if secondary unavailable
        selected_target = target1
        selected_angle = angle1
        selected_velocity = v1
        selected_ld = ld1
        used_plan = "primary"

    rospy.loginfo_throttle(
        0.1,
        "mode=%s blocked1=%s blocked2=%s"
        % (used_plan, blocked1, blocked2),
    )

    # Apply steering gain and clamp (pure pursuit steering unchanged)
    raw_angle = 5.0 * selected_angle
    angle_cmd = max(-STEERING_RANGE, min(STEERING_RANGE, raw_angle))
    command.steering_angle = angle_cmd

    # Final speed and lookahead update
    velocity = calculate_velocity(angle_cmd)
    lookahead_distance = adaptive_lookahead(velocity)
    command.speed = velocity

    command_pub.publish(command)

    # visualization points
    base = Point32(x=odom_x, y=odom_y)
    pose_pt = Point32(x=odom_x, y=odom_y)
    goal_pt = Point32(x=selected_target[0], y=selected_target[1])

    control_polygon.header.frame_id = frame_id
    control_polygon.header.seq = wp_seq
    control_polygon.header.stamp = rospy.Time.now()
    control_polygon.polygon.points = [pose_pt, base, goal_pt]

    wp_seq += 1

    polygon_pub.publish(control_polygon)
    # raceline_pub.publish(rvizrace.raceline_path)

    # ARROW MARKER
    arrow_marker = Marker()
    arrow_marker.header.frame_id = "map"
    arrow_marker.type = 0  # Arrow
    arrow_marker.header.stamp = rospy.Time.now()
    arrow_marker.id = 1
    arrow_marker.scale.x = 0.6
    arrow_marker.scale.y = 0.1
    arrow_marker.scale.z = 0.1

    arrow_marker.pose.position.x = odom_x
    arrow_marker.pose.position.y = odom_y
    arrow_marker.pose.position.z = 0.1

    quat = quaternion_from_euler(0, 0, math.radians(angle_cmd))
    arrow_marker.pose.orientation.x = quat[0]
    arrow_marker.pose.orientation.y = quat[1]
    arrow_marker.pose.orientation.z = quat[2]
    arrow_marker.pose.orientation.w = quat[3]

    arrow_marker.color.r = 0.0
    arrow_marker.color.g = 0.0
    arrow_marker.color.b = 1.0
    arrow_marker.color.a = 1.0

    arrow_marker_pub.publish(arrow_marker)

    if path_publish_times < 10:
        path_publish_times += 1
        publish_path()


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)

        if not plan:
            rospy.loginfo("Loading trajectory...")
            plan, path_resolution = construct_path(trajectory_name)
            plan2, path_resolution2 = construct_path(trajectory_name2)

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

        rospy.Subscriber("/%s/scan" % car_name, LaserScan, laser_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

