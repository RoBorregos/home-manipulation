#!/usr/bin/env python3

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
	s = rospy.Service('/cartesian_movement_services/Pick',Pick,handle_pick)
	print('Ready to execute Pick')

def handle_pick(req):
	print("Picking object")
	xarm.set_mode_cartesian()
	#vertical picks considers only X,Y,Z and yaw
	xarm.pick([req.object_pose[0],req.object_pose[1],req.object_pose[2],req.object_pose[3],req.object_pose[4],req.object_pose[5]],req.is_vertical,req.tip_pick)
	xarm.set_mode_moveit()
	return PickResponse(True)

def place_server():
	s = rospy.Service('/cartesian_movement_services/Place',Place,handle_place)
	print('Ready to execute Place')

def handle_place(req):
	print("Placing object")
	xarm.set_mode_cartesian()
	xarm.place([req.destination_pose[0],req.destination_pose[1],req.destination_pose[2],req.destination_pose[3],req.destination_pose[4],req.destination_pose[5]],req.is_vertical,req.tip_pick)
	xarm.set_mode_moveit()
	return PlaceResponse(True)

def pour_server():
	s = rospy.Service('/cartesian_movement_services/Pour',Pour,handle_pour)
	print('Ready to execute Pour')

def handle_pour(req):
	print("Pouring object")
	xarm.set_mode_cartesian()
	xarm.pour([req.pouring_point[0],req.pouring_point[1],req.pouring_point[2]],req.bowl_height,req.bowl_radius,req.object_height,req.grasp_height,req.left_to_right,req.tip_pick)
	xarm.set_mode_moveit()
	return PourResponse(True)

def move_joint_server():
	s = rospy.Service('/cartesian_movement_services/MoveJoint',MoveJoint,handle_move_joint)
	print('Ready to execute MoveJoint')

def handle_move_joint(req):
	print("Moving joints")
	xarm.set_mode_cartesian()
	xarm.move_joint(req.joint_number, m.radians(req.degree))
	xarm.set_mode_moveit()
	return MoveJointResponse(True)

if __name__ == "__main__":
	rospy.init_node('cartesian_server_2')
	xarm = arm()
	pick_server()
	place_server()
	pour_server()
	move_joint_server()
	rospy.spin()

