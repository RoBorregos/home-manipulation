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
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
	xarm.set_mode_cartesian()
	xarm.return_to_default_pose_horizontal()
	xarm.pick([req.object_pose[0],req.object_pose[1],req.object_pose[2]],req.is_vertical,req.tip_pick)
	return PickResponse(True)


if __name__ == "__main__":
	rospy.init_node('cartesian_server_2')
	xarm = arm()
	xarm.set_mode_cartesian()
	xarm.set_gripper(0)
	xarm.return_to_default_pose_horizontal()
	pick_server()
	rospy.spin()

