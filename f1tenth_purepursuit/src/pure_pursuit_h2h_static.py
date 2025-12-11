#!/usr/bin/env python
"""
Option 1: Static 3-Line Switch
- Pre-record 3 racelines: optimal, inside, outside
- Switch between them based on obstacle detection
- Simplest and most reliable approach
"""

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
from sensor_msgs.msg import LaserScan

plan = []
path_resolution = []
obstacle_points = []

frame_id = "map"
car_name = "car_4"
trajectory_name = "test_raceline"  # Start with optimal

# Overtaking parameters
LOOKAHEAD_CHECK = 3.0  # meters ahead to check for obstacles
SAFETY_DISTANCE = 0.6  # meters clearance from obstacles
SCAN_ANGLE_MIN = 20  # degrees (forward cone)
SCAN_ANGLE_MAX = 160  # degrees
MIN_OBSTACLE_DISTANCE = 2.5  # meters

command_pub = rospy.Publisher("/%s/offboard/command" % car_name, AckermannDrive, queue_size=100)
polygon_pub = rospy.Publisher("/%s/purepursuit_control/visualize" % car_name, PolygonStamped, queue_size=1)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)
path_pub = rospy.Publisher("/%s/purepursuit/path" % car_name, Path, queue_size=1)
arrow_marker_pub = rospy.Publisher("/arrow_marker", Marker, queue_size=20)

wp_seq = 0
control_polygon = PolygonStamped()

STEERING_RANGE = 100.0
WHEELBASE_LEN = 0.325


def _get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def construct_path(traj_name):
    """Load a specific trajectory CSV"""
    global plan, path_resolution
    plan = []
    path_resolution = []
    
    path_file = os.path.expanduser(
        "~/catkin_ws/src/f1tenth_purepursuit/racelines/%s.csv" % traj_name
    )

    if not os.path.exists(path_file):
        rospy.logerr("Path file not found: %s" % path_file)
        return False

    with open(path_file) as f:
        reader = csv.reader(f, delimiter=",")
        for wp in reader:
            plan.append([float(wp[0]), float(wp[1])])

    for i in range(1, len(plan)):
        dx = plan[i][0] - plan[i - 1][0]
        dy = plan[i][1] - plan[i - 1][1]
        path_resolution.append(math.sqrt(dx * dx + dy * dy))

    rospy.loginfo("Loaded %d waypoints from %s" % (len(plan), traj_name))
    return True


def laser_callback(scan_msg):
    """Process laser scan to detect obstacles"""
    global obstacle_points
    obstacle_points = []
    
    # Convert scan to cartesian coordinates
    angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
    
    for i, distance in enumerate(scan_msg.ranges):
        if not np.isfinite(distance) or distance < scan_msg.range_min or distance > scan_msg.range_max:
            continue
        
        angle_deg = math.degrees(angles[i])
        
        # Only consider points in forward cone
        if SCAN_ANGLE_MIN <= angle_deg <= SCAN_ANGLE_MAX:
            x = distance * np.cos(angles[i])
            y = distance * np.sin(angles[i])
            
            # Filter out very close points (likely walls) and far points
            if 0.3 < distance < MIN_OBSTACLE_DISTANCE:
                obstacle_points.append((x, y))


def is_path_blocked(path, odom_x, odom_y, heading):
    """Check if path has obstacles within lookahead distance"""
    if not obstacle_points:
        return False
    
    # Find closest point on path
    min_dist = float("inf")
    min_idx = 0
    for idx, pt in enumerate(path):
        d = _get_distance([odom_x, odom_y], pt)
        if d < min_dist:
            min_dist = d
            min_idx = idx
    
    # Check points ahead on the path
    distance = 0
    idx = min_idx
    prev_point = path[idx]
    
    while distance < LOOKAHEAD_CHECK and idx < len(path) - 1:
        idx += 1
        point = path[idx]
        distance += _get_distance(prev_point, point)
        
        # Check if any obstacle is too close to this waypoint
        for obs_x, obs_y in obstacle_points:
            # Transform obstacle to map frame
            obs_map_x = odom_x + obs_x * math.cos(heading) - obs_y * math.sin(heading)
            obs_map_y = odom_y + obs_x * math.sin(heading) + obs_y * math.cos(heading)
            
            dist = math.sqrt((point[0] - obs_map_x)**2 + (point[1] - obs_map_y)**2)
            if dist < SAFETY_DISTANCE:
                return True
        
        prev_point = point
    
    return False


def select_best_trajectory(odom_x, odom_y, heading):
    """Select best available trajectory"""
    global trajectory_name
    
    # Priority order: optimal > inside > outside
    trajectories = ["optimal", "inside", "outside"]
    
    for traj in trajectories:
        if construct_path(traj):
            if not is_path_blocked(plan, odom_x, odom_y, heading):
                if trajectory_name != traj:
                    rospy.loginfo("Switching to %s line" % traj)
                    trajectory_name = traj
                return True
    
    # If all blocked, stay on current and slow down
    rospy.logwarn("All paths blocked - staying on %s" % trajectory_name)
    return False


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


def purepursuit_control_node(data):
    global wp_seq

    if not plan:
        return

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    odom = [odom_x, odom_y]

    # Robot heading
    orientation_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )
    heading = tf.transformations.euler_from_quaternion(orientation_quat)[2]

    # Check and switch trajectories if needed
    select_best_trajectory(odom_x, odom_y, heading)

    command = AckermannDrive()

    # Find closest point on path
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

    max_vel = get_param_or_input("max_vel", 60.0, float)
    min_vel = get_param_or_input("min_vel", 30.0, float)

    # Speed based on steering
    def calculate_velocity(angle):
        a = abs(angle)
        angle_max = 40.0
        if a < 7.77:
            return max_vel
        if a > angle_max:
            return min_vel
        return (angle_max - a) / angle_max * (max_vel - min_vel) + min_vel

    # Lookahead based on speed
    def adaptive_lookahead(v):
        if (max_vel - min_vel) > 0:
            p = (v - min_vel) / (max_vel - min_vel)
        else:
            p = 0.0
        min_lookahead = 1.7
        max_lookahead = 2.7
        lookahead = min_lookahead + (max_lookahead - min_lookahead) * p
        return lookahead

    velocity = calculate_velocity(command.steering_angle)
    lookahead_distance = adaptive_lookahead(velocity)

    # Pick lookahead target point
    def get_target_point():
        idx = min_idx
        while idx < len(plan) - 1:
            if _get_distance(odom, plan[idx]) >= lookahead_distance:
                break
            idx += 1
        return plan[max(0, idx - 1)]

    target_pt = get_target_point()

    # Steering calculation
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
    angle = 5 * s_angle
    angle = max(-STEERING_RANGE, min(STEERING_RANGE, angle))

    command.steering_angle = angle

    velocity = calculate_velocity(command.steering_angle)
    command.speed = velocity

    # Slow down if obstacles nearby
    if obstacle_points:
        command.speed *= 0.85

    rospy.loginfo_throttle(0.5,
        "traj=%s obstacles=%d v=%.1f angle=%.1f" %
        (trajectory_name, len(obstacle_points), command.speed, command.steering_angle)
    )

    command_pub.publish(command)

    # Visualization
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

    # Arrow marker
    arrow_marker = Marker()
    arrow_marker.header.frame_id = "map"
    arrow_marker.type = 0
    arrow_marker.header.stamp = rospy.Time.now()
    arrow_marker.id = 1
    arrow_marker.scale.x = 0.6
    arrow_marker.scale.y = 0.1
    arrow_marker.scale.z = 0.1

    arrow_marker.pose.position.x = odom_x
    arrow_marker.pose.position.y = odom_y
    arrow_marker.pose.position.z = 0.1
    
    quat = quaternion_from_euler(0, 0, math.radians(s_angle))
    arrow_marker.pose.orientation.x = quat[0]
    arrow_marker.pose.orientation.y = quat[1]
    arrow_marker.pose.orientation.z = quat[2]
    arrow_marker.pose.orientation.w = quat[3]

    arrow_marker.color.r = 0.0
    arrow_marker.color.g = 0.0
    arrow_marker.color.b = 1.0
    arrow_marker.color.a = 1.0

    arrow_marker_pub.publish(arrow_marker)


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)

        rospy.loginfo("Loading trajectory...")
        if construct_path(trajectory_name):
            publish_path()
            rospy.loginfo("Trajectory loaded.")
        else:
            rospy.logerr("No trajectory loaded. Check CSV path.")

        # Subscribe to laser scan for obstacle detection
        rospy.Subscriber("/%s/scan" % car_name, LaserScan, laser_callback)
        
        rospy.Subscriber(
            "/%s/particle_filter/viz/inferred_pose" % car_name,
            PoseStamped,
            purepursuit_control_node
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass