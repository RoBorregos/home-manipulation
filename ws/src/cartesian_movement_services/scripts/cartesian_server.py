#!/usr/bin/env python3

"""
This script provides a set of services to control the xArm robot using ROS.
It includes services for picking, placing, pouring, and moving joints.
"""

from __future__ import print_function
from cartesian_movement_services.srv import *
from xarm_msgs.srv import *
import rospkg
import importlib
import sys
import time
import rospy
import math as m
import copy
import rospy
from cartesian_movement_services.srv import *
from xarm_msgs.srv import *
from arm_movements import arm
import rospy

##########Definition of arm services####################

def pick_server():
    """
    Initialize the Pick service server.
    """
    s = rospy.Service('/cartesian_movement_services/Pick', Pick, handle_pick)
    print('Ready to execute Pick')

def handle_pick(req):
    """
    Handle the Pick service request.
    """
    print("Picking object")
    xarm.set_mode_cartesian()
    # vertical picks considers only X, Y, Z and yaw
    xarm.pick([req.object_pose[0], req.object_pose[1], req.object_pose[2], req.object_pose[3], req.object_pose[4], req.object_pose[5]], req.is_vertical, req.tip_pick)
    xarm.set_mode_moveit()
    return PickResponse(True)

def place_server():
    """
    Initialize the Place service server.
    """
    s = rospy.Service('/cartesian_movement_services/Place', Place, handle_place)
    print('Ready to execute Place')

def handle_place(req):
    """
    Handle the Place service request.
    """
    print("Placing object")
    xarm.set_mode_cartesian()
    xarm.place([req.destination_pose[0], req.destination_pose[1], req.destination_pose[2], req.destination_pose[3], req.destination_pose[4], req.destination_pose[5]], req.is_vertical, req.tip_pick)
    xarm.set_mode_moveit()
    return PlaceResponse(True)

def pour_server():
    """
    Initialize the Pour service server.
    """
    s = rospy.Service('/cartesian_movement_services/Pour', Pour, handle_pour)
    print('Ready to execute Pour')

def handle_pour(req):
    """
    Handle the Pour service request.
    """
    print("Pouring object")
    xarm.set_mode_cartesian()
    xarm.pour([req.pouring_point[0], req.pouring_point[1], req.pouring_point[2]], req.bowl_height, req.bowl_radius, req.object_height, req.grasp_height, req.left_to_right, req.tip_pick)
    xarm.set_mode_moveit()
    return PourResponse(True)

def move_joint_server():
    """
    Initialize the MoveJoint service server.
    """
    s = rospy.Service('/cartesian_movement_services/MoveJoint', MoveJoint, handle_move_joint)
    print('Ready to execute MoveJoint')

def handle_move_joint(req):
    """
    Handle the MoveJoint service request.
    """
    print("Moving joints")
    xarm.set_mode_cartesian()
    xarm.move_joint(req.joint_number, m.radians(req.degree))
    xarm.set_mode_moveit()
    return MoveJointResponse(True)

if __name__ == "__main__":
    rospy.init_node('cartesian_server_2')
    xarm = arm()
    xarm.set_mode_moveit()
    pick_server()
    place_server()
    pour_server()
    move_joint_server()
    rospy.spin()

# Examples and use cases for key technologies used

# Example of using rospy to create a ROS service for arm movements
def example_rospy_service():
    rospy.init_node('example_service')
    service = rospy.Service('example_service', SetInt16, handle_example_service)
    rospy.spin()

def handle_example_service(req):
    print("Handling example service request")
    return SetInt16Response(True)

# Example of using xarm_msgs to send commands to the xArm robot
def example_xarm_command():
    rospy.wait_for_service('/xarm/move_line')
    move_line = rospy.ServiceProxy('/xarm/move_line', Move)
    req = MoveRequest()
    req.pose = [0, 0, 0, 0, 0, 0]
    req.mvvelo = 100
    req.mvacc = 200
    req.mvtime = 0
    move_line(req)

# Example of using math library for calculations
def example_math_calculation():
    angle = m.radians(45)
    print("Angle in radians:", angle)
