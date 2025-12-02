#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import numpy as np
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
import tf
import rvizrace

from nav_msgs.msg import Path

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
# car_name            = str(sys.argv[1])
# trajectory_name     = str(sys.argv[2])
car_name            = 'car_4'
trajectory_name     = 'raceline'

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 10)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)
raceline_pub = rospy.Publisher("/raceline", Path, queue_size=1)

# visualize refrence path
path_pub = rospy.Publisher('/{}/purepursuit/path'.format(car_name), Path, queue_size=1)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq          = 0
control_polygon = PolygonStamped()

def _get_distance(point1, point2):
    # eucliodian distance
    x1, y1 = point1[0], point1[1]
    x2, y2 = point2[0], point2[1]
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('~/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])


    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))



# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y


    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
    odom = [odom_x, odom_y]
    # Your code here
    def get_base_projection():
        min_dist = 999
        min_idx = 0 
        # min_pt = (0, 0)
        for idx, pt in enumerate(plan):
            distance = _get_distance(odom, pt)
            if distance < min_dist :
                min_dist = distance
                min_idx = idx
                # min_pt = (pt[0], pt[1]) # x,y
        return min_idx
    
    min_idx = get_base_projection()

    
    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]
    

    # TODO 2: You need to tune the value of the lookahead_distance
    lookahead_distance = 2.0


    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    # Your code here
    def get_target_point():
        idx = min_idx
        while idx < len(plan) - 1:
            distance = _get_distance(odom, plan[idx])
            if distance >= lookahead_distance:
                break
            idx += 1
        x = plan[idx - 1][0]
        y = plan[idx - 1][1]
        target_pt = (x, y)
        # we could make this interpolate, so we dont have points behind corners
        # in class it was discussed that we could also just have more waypoints to prevent that issue
        return target_pt
    
    target_pt = get_target_point()

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here
    def calculate_steering():
        # alpha = arcsin(y_t / l_d)
        # delta = arctan(2L * sin(alpha) / l_d)

        # Basically, the pure pursuit angle calcs expects y_t to be in 
        # the robot's frame, so we have to find the delta position
        # from the center of the car to the target point then
        # do an anti-rotation by the car's pose
        carFrameTarget = tf.transformations.quaternion_from_euler(0, 0, 0)
        carFrameTarget[0] = target_pt[0] - odom[0]
        carFrameTarget[1] = target_pt[1] - odom[1]
        carFrameTarget[2] = 0
        carFrameTarget[3] = 0
        orientation = tf.transformations.quaternion_from_euler(0, 0, 0)
        orientation[0] = data.pose.orientation.x
        orientation[1] = data.pose.orientation.y
        orientation[2] = data.pose.orientation.z
        orientation[3] = data.pose.orientation.w
        conj = tf.transformations.quaternion_conjugate(orientation)
        carFrameTarget = tf.transformations.quaternion_multiply(conj, carFrameTarget)
        carFrameTarget = tf.transformations.quaternion_multiply(carFrameTarget, orientation)

        alpha = math.asin((carFrameTarget[1]) / lookahead_distance)
        # print("Angle: ", math.degrees(alpha))
        steering_angle = math.atan(2 * WHEELBASE_LEN * math.sin(alpha) / lookahead_distance)
        steering_angle = math.degrees(steering_angle)
        return steering_angle


    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here    
    angle = 2.5 * calculate_steering()
    command.steering_angle = angle
    # print("Steering at: ", command.steering_angle)

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    def calculate_velocity(angle):
        angle = abs(angle)
        # dynamic velocity based on steering angle
        min_vel = 30.0
        max_vel = 60.0

        angleMax = 50.0

        if angle > angleMax:
            return min_vel
        
        velocity = (angleMax - angle) / angleMax * (max_vel - min_vel) + min_vel

        return velocity

    velocity = calculate_velocity(command.steering_angle)
    command.speed = velocity
    # command.speed = 30.0
    command_pub.publish(command)


    # print("DATA: ")
    # print(data)
    # print(target_pt)
    # print(odom)
    # print("angle", angle)
    # print("speed", velocity)


    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

    # These are set to zero only so that the template code builds. 
    pose_x=odom_x    
    pose_y=odom_y
    target_x=target_pt[0]
    target_y=target_pt[1]


    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)
    raceline_pub.publish(rvizrace.raceline_path)

def publish_path():
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for p in plan:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    path_pub.publish(path_msg)

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
            publish_path()
            rospy.loginfo('trajectory done!')

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass


