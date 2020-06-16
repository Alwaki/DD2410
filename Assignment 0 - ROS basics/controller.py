#!/usr/bin/env python2

# School: Royal Institute of Technology
# Course: DD2410
# Work: Laboration 0 - Introduction to ROS
# Author: Alexander Wallén Kiessling
# Date: 2 June 2020
# Summary: This script is intended for control of a turtlebot 3 through ROS
# functionality.

# SETUP
# ---------------------

# Import packages
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Declare variables
# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
action_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5 # tried higher speeds, resulted in drifting
# Max angular velocity (rad/s)
max_angular_velocity = 1.0

# FUNCTION DEFINITIONS
# ---------------------
# Description: "move" is a function that takes a given path, and converts it into
# information for movement to the robot
# In: custom struct type "path" which contains pose vector
# Out: void type functionality, no direct return. Publishes custom struct "twist"
def move(path):
    # Include variables from global scope to function scope
    global control_client, robot_frame_id, pub

    # Create instance of twist class
    twist_msg = Twist()

    # Loop whilst pose vector from path struct is not empty
    while path.poses:
        # Call service client with path
        result = control_client(path)
        setpoint = result.setpoint
        new_path = result.new_path

        # Transform Setpoint from service client (see tutorial for tf2)
        transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time())
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        # Create Twist message from the transformed Setpoint
        twist_msg.angular.z = min(atan2(transformed_setpoint.point.y, transformed_setpoint.point.x), max_angular_velocity)
        twist_msg.linear.x = min(hypot(transformed_setpoint.point.x, transformed_setpoint.point.y), max_linear_velocity)

        # Limit the linear speed if robot needs to turn hard (avoids wall collisions)
        if twist_msg.angular.z > 0.75:
            twist_msg.linear.x = 0

        # Publish Twist (to velocity topic)
        pub.publish(twist_msg)
        rate.sleep()

        # Call service client again if the returned path is not empty and do stuff again
        path = new_path

    # Send 0 control Twist to stop robot when loop finished
    twist_msg.angular.z = 0
    twist_msg.linear.x = 0
    pub.publish(twist_msg)
    rate.sleep()

    # Get new path from action server
    get_path()

# Description: "get_path" is a function which communicates with action server
# through goal/result interaction. This makes use of the actionlib library.
# In: n/a
# Out: void type functionality, instead calls "move" function. (Note recursion)
def get_path():
    # Include variables from global scope to function scope
    global action_client

    # Get path from action server
    action_client.wait_for_server() #Blocks until the action server connects to this client
    goal = irob_assignment_1.msg.GetNextGoalAction()
    action_client.send_goal(goal)
    action_client.wait_for_result()

    # Call move with path from action server
    move(action_client.get_result().path)

# DRIVER CODE
# ---------------------

if __name__ == "__main__":
    # Init node
    rospy.init_node("controller") # Only one node can be initialized at one time.
    # Init publisher
    rate = rospy.Rate(10.0) # 10Hz
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10) # queue size determines how many messages are stored
    # Init simple action client
    action_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)
    # Setup tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()

    # Spin
    rospy.spin()
