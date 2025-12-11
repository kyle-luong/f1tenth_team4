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
from sensor_msgs.msg import LaserScan

plan = []
original_plan = []
path_resolution = []
obstacle_points = []

# State machine
overtake_state = "following"  # following, preparing_overtake, overtaking, returning
obstacle_cleared_idx = 0

frame_id = "map"
car_name = "car_4"
trajectory_name = "working_raceline"

# Overtaking parameters
OFFSET_DISTANCE = 0.5  # 0.5
OBSTACLE_DETECT_DISTANCE = 1.5  # 1.5
OBSTACLE_CLEARED_DISTANCE = 1.5  # 1.5
RETURN_BLEND_WAYPOINTS = 20     # 20
SCAN_ANGLE_MIN = -15
SCAN_ANGLE_MAX = 15

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


def laser_callback(scan_msg):
    """Process laser scan to detect obstacles"""
    global obstacle_points
    obstacle_points = []
    
    angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
    
    for i, distance in enumerate(scan_msg.ranges):
        if not np.isfinite(distance) or distance < scan_msg.range_min or distance > scan_msg.range_max:
            continue
        
        if i >= len(angles):
            return
        angle_deg = math.degrees(angles[i])
        
        if SCAN_ANGLE_MIN <= angle_deg <= SCAN_ANGLE_MAX:
            x = distance * np.cos(angles[i])
            y = distance * np.sin(angles[i])
            
            if 0.3 < distance < OBSTACLE_DETECT_DISTANCE:
                obstacle_points.append((x, y))


def is_obstacle_ahead(odom_x, odom_y, heading):
    """Check if there's an obstacle directly ahead"""
    if not obstacle_points:
        return False
    
    for obs_x, obs_y in obstacle_points:
        # Transform to map frame
        obs_map_x = odom_x + obs_x * math.cos(heading) - obs_y * math.sin(heading)
        obs_map_y = odom_y + obs_x * math.sin(heading) + obs_y * math.cos(heading)
        
        # Check if in front cone
        dist = math.sqrt((obs_map_x - odom_x)**2 + (obs_map_y - odom_y)**2)
        if dist < OBSTACLE_DETECT_DISTANCE:
            # print("dist: ", dist , "OBSTACLE_DETECT_DISTANCE: ", OBSTACLE_DETECT_DISTANCE)
            return True
    
    return False


def generate_offset_line(base_path, start_idx, offset_distance, num_points=50):
    """Generate offset line starting from start_idx"""
    offset_path = []
    
    end_idx = min(start_idx + num_points, len(base_path))
    
    for i in range(start_idx, end_idx):
        if i == 0:
            dx = base_path[i+1][0] - base_path[i][0]
            dy = base_path[i+1][1] - base_path[i][1]
        elif i >= len(base_path) - 1:
            dx = base_path[i][0] - base_path[i-1][0]
            dy = base_path[i][1] - base_path[i-1][1]
        else:
            dx = base_path[i+1][0] - base_path[i-1][0]
            dy = base_path[i+1][1] - base_path[i-1][1]
        
        length = math.sqrt(dx*dx + dy*dy)
        if length < 0.001:
            offset_path.append(base_path[i][:])
            continue
            
        dx /= length
        dy /= length
        
        # Left offset (counter-clockwise)
        perp_x = -dy
        perp_y = dx
        
        new_x = base_path[i][0] + perp_x * offset_distance
        new_y = base_path[i][1] + perp_y * offset_distance
        
        offset_path.append([new_x, new_y])
    
    return offset_path


def blend_paths(current_x, current_y, target_path, closest_idx, blend_points):
    """Create smooth blended path from current position to target path"""
    blended = []
    
    # Start from current position
    start_point = [current_x, current_y]
    
    # End on target path ahead
    end_idx = min(closest_idx + blend_points, len(target_path) - 1)
    end_point = target_path[end_idx]
    
    # Create smooth curve using linear interpolation
    for i in range(blend_points):
        t = float(i) / float(blend_points - 1) if blend_points > 1 else 0
        
        x = (1 - t) * start_point[0] + t * end_point[0]
        y = (1 - t) * start_point[1] + t * end_point[1]
        
        blended.append([x, y])
    
    # Add rest of target path
    blended.extend(target_path[end_idx:])
    
    return blended


def get_closest_index_on_path(odom_x, odom_y, path):
    """Get closest waypoint index"""
    min_dist = float("inf")
    min_idx = 0
    
    for idx, pt in enumerate(path):
        d = _get_distance([odom_x, odom_y], pt)
        if d < min_dist:
            min_dist = d
            min_idx = idx
    
    return min_idx


def distance_to_path(odom_x, odom_y, path):
    """Get minimum distance to path"""
    min_dist = float("inf")
    
    for pt in path:
        d = _get_distance([odom_x, odom_y], pt)
        if d < min_dist:
            min_dist = d
    
    return min_dist


def state_machine_update(odom_x, odom_y, heading):
    """Update state machine and path"""
    global plan, overtake_state, obstacle_cleared_idx
    
    if overtake_state == "following":
        if is_obstacle_ahead(odom_x, odom_y, heading):
            # Obstacle detected - prepare overtake
            rospy.loginfo("Obstacle detected - preparing overtake")
            overtake_state = "preparing_overtake"
            
            # Generate offset path
            closest_idx = get_closest_index_on_path(odom_x, odom_y, original_plan)
            plan = generate_offset_line(original_plan, closest_idx, OFFSET_DISTANCE, num_points=60)
    
    elif overtake_state == "preparing_overtake":
        if not is_obstacle_ahead(odom_x, odom_y, heading):
            # Obstacle cleared - now overtaking
            rospy.loginfo("Obstacle cleared - overtaking")
            overtake_state = "overtaking"
            obstacle_cleared_idx = get_closest_index_on_path(odom_x, odom_y, original_plan)

    elif overtake_state == "overtaking":

        if is_obstacle_ahead(odom_x, odom_y, heading):
            # New car in the overtake path - abort overtake
            rospy.loginfo("New obstacle in overtake lane - aborting overtake")
            overtake_state = "returning"
            closest_idx = get_closest_index_on_path(odom_x, odom_y, original_plan)
            plan = blend_paths(odom_x, odom_y, original_plan, closest_idx, RETURN_BLEND_WAYPOINTS)
        else:
            current_idx = get_closest_index_on_path(odom_x, odom_y, original_plan)
            if current_idx > obstacle_cleared_idx + 15:
                # Safe to return to racing line
                rospy.loginfo("Returning to racing line")
                overtake_state = "returning"
                
                # Generate smooth return path
                closest_idx = get_closest_index_on_path(odom_x, odom_y, original_plan)
                plan = blend_paths(odom_x, odom_y, original_plan, closest_idx, RETURN_BLEND_WAYPOINTS)
    
    elif overtake_state == "returning":
        # Check if we're back on racing line
        dist = distance_to_path(odom_x, odom_y, original_plan)
        
        if dist < 0.4:
            # Back on line
            rospy.loginfo("Back on racing line - following")
            overtake_state = "following"
            plan = [pt[:] for pt in original_plan]


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

    orientation_quat = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    )
    heading = tf.transformations.euler_from_quaternion(orientation_quat)[2]

    # Update state machine
    # state_machine_update(odom_x, odom_y, heading)

    command = AckermannDrive()

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
        min_lookahead = 0.2
        max_lookahead = 2
        lookahead = min_lookahead + (max_lookahead - min_lookahead) * p
        return lookahead

    def obstacle_speed_cap():
        """
        Limit max speed based on distance to the nearest obstacle in front.
        Uses obstacle_points already in car frame (x forward, y left).
        """
        if not obstacle_points:
            return max_vel

        min_forward_dist = None

        for obs_x, obs_y in obstacle_points:
            if obs_x <= 0.0:
                continue
            d = math.hypot(obs_x, obs_y)
            if (min_forward_dist is None) or (d < min_forward_dist):
                min_forward_dist = d

        if min_forward_dist is None:
            return max_vel

        if min_forward_dist < 1.1:
            return 0.0
        elif min_forward_dist < 1.7:
            return 15.0
        elif min_forward_dist < 2.0:
            return min_vel
        else:
            return max_vel
        
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

        y_c = car_frame[1]

        alpha = math.asin(max(-1.0, min(1.0, y_c / lookahead_distance)))
        steer = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / lookahead_distance)
        angle = math.degrees(steer)
        
        return angle

    s_angle = calculate_steering()
    angle = 5 * s_angle
    angle = max(-STEERING_RANGE, min(STEERING_RANGE, angle))

    command.steering_angle = angle
    velocity = calculate_velocity(command.steering_angle)
    lookahead_distance = adaptive_lookahead(velocity)
    command.speed = velocity

    # Adjust speed based on state
    # if overtake_state == "preparing_overtake":
    #     command.speed *= 0.9  # Slow down slightly
    # elif overtake_state == "overtaking":
    #     command.speed *= 1.05  # Push harder to complete pass

    velocity = min(command.speed, obstacle_speed_cap())   
    command.speed = velocity
    lookahead_distance = adaptive_lookahead(velocity)

    rospy.loginfo_throttle(0.5,
        "state=%s obstacles=%d v=%.1f angle=%.1f" %
        (overtake_state, len(obstacle_points), command.speed, command.steering_angle)
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
