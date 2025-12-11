#!/usr/bin/env python
"""
Option 4: Pure Pursuit + Follow The Gap Hybrid
- Follows optimal raceline when clear
- Switches to Follow-The-Gap when obstacles block all paths
- Most reactive approach for tight situations
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
original_plan = []
path_resolution = []
current_scan = None
drive_mode = "pure_pursuit"  # pure_pursuit or ftg

frame_id = "map"
car_name = "car_4"
trajectory_name = "optimal"

# Control parameters
OFFSET_DISTANCE = 0.7
LOOKAHEAD_CHECK = 3.0
SAFETY_DISTANCE = 0.6
FTG_BUBBLE_RADIUS = 0.3  # Radius to clear around obstacles
FTG_MAX_GAP_THRESHOLD = 0.5  # Minimum gap width to consider
OBSTACLE_THRESHOLD = 2.0  # Distance to switch to FTG

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


def construct_path():
    """Load the base optimal trajectory"""
    global original_plan, plan, path_resolution
    original_plan = []
    plan = []
    path_resolution = []
    
    path_file = os.path.expanduser(
        "~/catkin_ws/src/f1tenth_purepursuit/racelines/%s.csv" % trajectory_name
    )

    if not os.path.exists(path_file):
        rospy.logerr("Path file not found: %s" % path_file)
        return

    with open(path_file) as f:
        reader = csv.reader(f, delimiter=",")
        for wp in reader:
            original_plan.append([float(wp[0]), float(wp[1])])

    plan = [pt[:] for pt in original_plan]

    for i in range(1, len(plan)):
        dx = plan[i][0] - plan[i - 1][0]
        dy = plan[i][1] - plan[i - 1][1]
        path_resolution.append(math.sqrt(dx * dx + dy * dy))

    rospy.loginfo("Loaded %d waypoints" % len(plan))


def generate_offset_line(base_path, offset_distance, side='left'):
    """Generate parallel line offset from base path"""
    offset_path = []
    
    for i in range(len(base_path)):
        if i == 0:
            dx = base_path[i+1][0] - base_path[i][0]
            dy = base_path[i+1][1] - base_path[i][1]
        else:
            dx = base_path[i][0] - base_path[i-1][0]
            dy = base_path[i][1] - base_path[i-1][1]
        
        length = math.sqrt(dx*dx + dy*dy)
        if length < 0.001:
            offset_path.append(base_path[i][:])
            continue
            
        dx /= length
        dy /= length
        
        if side == 'left':
            perp_x = -dy
            perp_y = dx
        else:
            perp_x = dy
            perp_y = -dx
        
        new_x = base_path[i][0] + perp_x * offset_distance
        new_y = base_path[i][1] + perp_y * offset_distance
        
        offset_path.append([new_x, new_y])
    
    return offset_path


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


def find_max_gap(free_space_ranges):
    """Find the largest gap in free space"""
    masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
    slices = np.ma.notmasked_contiguous(masked)
    
    if slices is None or len(slices) == 0:
        return None, None
    
    max_len = 0
    max_slice = None
    
    for s in slices:
        slice_len = s.stop - s.start
        if slice_len > max_len:
            max_len = slice_len
            max_slice = s
    
    if max_slice is None:
        return None, None
    
    # Return center of the gap
    center_idx = (max_slice.start + max_slice.stop) // 2
    return center_idx, max_len


def ftg_steering_angle(scan):
    """Follow The Gap algorithm - returns steering angle in degrees"""
    if scan is None:
        return 0.0
    
    # Preprocess
    proc_ranges = preprocess_lidar(scan.ranges)
    proc_ranges = np.array(proc_ranges)
    
    # Find closest point
    closest_idx = np.argmin(proc_ranges)
    min_dist = proc_ranges[closest_idx]
    
    # Create safety bubble around closest obstacle
    if min_dist < OBSTACLE_THRESHOLD:
        bubble_indices = int(FTG_BUBBLE_RADIUS / (min_dist * scan.angle_increment))
        start_idx = max(0, closest_idx - bubble_indices)
        end_idx = min(len(proc_ranges), closest_idx + bubble_indices)
        proc_ranges[start_idx:end_idx] = 0.0
    
    # Find max gap
    gap_idx, gap_len = find_max_gap(proc_ranges)
    
    if gap_idx is None:
        rospy.logwarn("FTG: No gap found")
        return 0.0
    
    # Convert gap index to angle
    angle = scan.angle_min + gap_idx * scan.angle_increment
    steering_angle = math.degrees(angle)
    
    # Clip to reasonable range for pure pursuit compatibility
    steering_angle = max(-30.0, min(30.0, steering_angle))
    
    return steering_angle


def is_path_blocked(path, odom_x, odom_y, heading, scan):
    """Check if path has obstacles using LiDAR scan"""
    if scan is None:
        return False
    
    # Find closest point on path
    min_dist = float("inf")
    min_idx = 0
    for idx, pt in enumerate(path):
        d = _get_distance([odom_x, odom_y], pt)
        if d < min_dist:
            min_dist = d
            min_idx = idx
    
    # Check points ahead on path
    distance = 0
    idx = min_idx
    prev_point = path[idx]
    
    while distance < LOOKAHEAD_CHECK and idx < len(path) - 1:
        idx += 1
        point = path[idx]
        distance += _get_distance(prev_point, point)
        
        # Check if any scan point is near this waypoint
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        
        for i, r in enumerate(scan.ranges):
            if np.isinf(r) or np.isnan(r) or r > OBSTACLE_THRESHOLD:
                continue
            
            # Convert scan point to map frame
            obs_x = r * math.cos(angles[i])
            obs_y = r * math.sin(angles[i])
            
            obs_map_x = odom_x + obs_x * math.cos(heading) - obs_y * math.sin(heading)
            obs_map_y = odom_y + obs_x * math.sin(heading) + obs_y * math.cos(heading)
            
            dist = math.sqrt((point[0] - obs_map_x)**2 + (point[1] - obs_map_y)**2)
            if dist < SAFETY_DISTANCE:
                return True
        
        prev_point = point
    
    return False


def select_drive_mode(odom_x, odom_y, heading):
    """Decide whether to use Pure Pursuit or FTG"""
    global plan, drive_mode
    
    if current_scan is None or not original_plan:
        return
    
    # Check if optimal line is clear
    optimal_blocked = is_path_blocked(original_plan, odom_x, odom_y, heading, current_scan)
    
    if not optimal_blocked:
        # Use pure pursuit on optimal line
        if drive_mode != "pure_pursuit":
            rospy.loginfo("Switching to Pure Pursuit - path clear")
            drive_mode = "pure_pursuit"
        plan = [pt[:] for pt in original_plan]
        return
    
    # Try offset lines
    left_line = generate_offset_line(original_plan, OFFSET_DISTANCE, side='left')
    if not is_path_blocked(left_line, odom_x, odom_y, heading, current_scan):
        if drive_mode != "pure_pursuit":
            rospy.loginfo("Switching to Pure Pursuit - left line clear")
            drive_mode = "pure_pursuit"
        plan = left_line
        return
    
    right_line = generate_offset_line(original_plan, OFFSET_DISTANCE, side='right')
    if not is_path_blocked(right_line, odom_x, odom_y, heading, current_scan):
        if drive_mode != "pure_pursuit":
            rospy.loginfo("Switching to Pure Pursuit - right line clear")
            drive_mode = "pure_pursuit"
        plan = right_line
        return
    
    # All paths blocked - switch to FTG
    if drive_mode != "ftg":
        rospy.loginfo("Switching to FTG - all paths blocked")
        drive_mode = "ftg"


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

    if not plan or current_scan is None:
        return

    odom_x = data.pose.position.x
    odom_y = data.pose.position.y
    odom = [odom_x, odom_y]

    orientation_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )
    heading = tf.transformations.euler_from_quaternion(orientation_quat)[2]

    # Select drive mode
    select_drive_mode(odom_x, odom_y, heading)

    command = AckermannDrive()

    max_vel = get_param_or_input("max_vel", 60.0, float)
    min_vel = get_param_or_input("min_vel", 30.0, float)

    if drive_mode == "ftg":
        # Follow The Gap mode
        ftg_angle = ftg_steering_angle(current_scan)
        
        # Scale to steering range
        angle = ftg_angle * 5.0  # Similar scaling to pure pursuit
        angle = max(-STEERING_RANGE, min(STEERING_RANGE, angle))
        
        command.steering_angle = angle
        
        # Conservative speed in FTG mode
        command.speed = min_vel * 1.2
        
        rospy.loginfo_throttle(0.5,
            "FTG mode: angle=%.1f speed=%.1f" % (command.steering_angle, command.speed)
        )
    
    else:
        # Pure Pursuit mode
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

        def calculate_velocity(angle):
            a = abs(angle)
            angle_max = 40.0
            if a < 7.77:
                return max_vel
            if a > angle_max:
                return min_vel
            return (angle_max - a) / angle_max * (max_vel - min_vel) + min_vel

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

        rospy.loginfo_throttle(0.5,
            "PP mode: v=%.1f angle=%.1f" % (command.speed, command.steering_angle)
        )

    command_pub.publish(command)

    # Visualization
    if drive_mode == "pure_pursuit":
        base = Point32(x=odom_x, y=odom_y)
        pose_pt = Point32(x=odom_x, y=odom_y)
        
        if len(plan) > 0:
            # Find target point for visualization
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
            target_pt = plan[min(min_idx + 10, len(plan) - 1)]
            goal_pt = Point32(x=target_pt[0], y=target_pt[1])
        else:
            goal_pt = Point32(x=odom_x, y=odom_y)

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
    
    quat = quaternion_from_euler(0, 0, math.radians(command.steering_angle / 5.0))
    arrow_marker.pose.orientation.x = quat[0]
    arrow_marker.pose.orientation.y = quat[1]
    arrow_marker.pose.orientation.z = quat[2]
    arrow_marker.pose.orientation.w = quat[3]

    # Color based on mode
    if drive_mode == "ftg":
        arrow_marker.color.r = 1.0  # Red for FTG
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
    else:
        arrow_marker.color.r = 0.0  # Blue for Pure Pursuit
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 1.0
    arrow_marker.color.a = 1.0

    arrow_marker_pub.publish(arrow_marker)


if __name__ == "__main__":
    try:
        rospy.init_node("pure_pursuit", anonymous=True)

        rospy.loginfo("Loading trajectory...")
        construct_path()

        if plan:
            publish_path()
            rospy.loginfo("Trajectory loaded.")
        else:
            rospy.logerr("No trajectory loaded. Check CSV path.")

        rospy.Subscriber("/%s/scan" % car_name, LaserScan, laser_callback)
        
        rospy.Subscriber(
            "/%s/particle_filter/viz/inferred_pose" % car_name,
            PoseStamped,
            purepursuit_control_node
        )

        rospy.spin()

    except rospy.ROSInterruptException:
        pass