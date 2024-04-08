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
rospack = rospkg.RosPack()
rospack.list()
path = rospack.get_path('cartesian_movement_services')
MODULE_PATH = path+"/arm_scripts/pouring_srv.py"
MODULE_NAME = "Arm_module"
spec = importlib.util.spec_from_file_location(MODULE_NAME, MODULE_PATH)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module 
spec.loader.exec_module(module)
module = importlib.import_module("Arm_module")

##########Definition of arm services####################

#End effector rotation server
def handle_move_end_effector(req):
	print('Moving end effector')	
	module.adjust_end_effector_yaw(req.degree)
	return TurnEndEffectorResponse(True)

def move_end_effector_server():
	s = rospy.Service('/cartesian_movement_services/TurnEndEffector',TurnEndEffector,handle_move_end_effector)
	print('Ready to move end effector')

#Pick and place server
def handle_pick_and_place(req):
	print('Executing pick and place')
	print(req)
	succesful = False
	#vertical_pick_and_place(req.object_pose[0],req.object_pose[1],req.object_pose[2] + 175,req.object_pose[5],req.destination_pose[0],req.destination_pose[1],req.destination_pose[2]+175,req.destination_pose[5])
	#vertical_pick_and_place(req.object_pose[0],req.object_pose[1],req.object_pose[2]+175,req.object_pose[5],req.object_pose[0],req.object_pose[1],req.object_pose[2]+175,req.object_pose[5])
	try:
		if(req.is_vertical == True):
			print('Vertical pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Z axis must be increased by 175mm to avoid the tip of the end effector to crush itself with the table
				grasping_z_axis = req.object_pose[2] + 175
				module.vertical_pick_and_place(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5],req.destination_pose[0],req.destination_pose[1],grasping_z_axis,req.destination_pose[5])
				return PickAndPlaceResponse(True)
			else:
				print('Middle pick')
				#The Z axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_z_axis = req.object_pose[2] + 135
				module.vertical_pick_and_place(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5],req.destination_pose[0],req.destination_pose[1],grasping_z_axis,req.destination_pose[5])
				return PickAndPlaceResponse(True)
			#vertical_pick_and_place(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5],req.destiantion_pose[0],req.destination_pose[1],req.destination_pose[2],req.destination_pose[5])
		else:
			print('Horizontal pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Y axis must be increase	d by 175mm to make the tip of the end effector to be in the same position as the object
				grasping_Y_axis = req.object_pose[1] + 175
				module.horizontal_pick_and_place(req.object_pose[0],grasping_Y_axis,req.object_pose[2],req.destination_pose[0],grasping_Y_axis,req.destination_pose[2])
				return PickAndPlaceResponse(True)
			else:
				print('Middle pick')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_Y_axis = req.object_pose[1] + 135
				module.horizontal_pick_and_place(req.object_pose[0],grasping_Y_axis,req.object_pose[2],req.destination_pose[0],grasping_Y_axis,req.destination_pose[2])
				return PickAndPlaceResponse(True)
	except:
		print('Pick and place failed')
		return PickAndPlaceResponse(False)

def pick_and_place_server():
	s = rospy.Service('/cartesian_movement_services/PickAndPlace',PickAndPlace,handle_pick_and_place)
	print('Ready to execute Pick and Place')

#Pouring server
def handle_pick_and_pour(req):
	module.return_to_default_pose_horizontal()
	print('Executing pick and pour')
	print(req)
	# pick_and_pour(0,-330,380,10,-330,380,21,85,70)
	#pick_and_pour(req.object_pose[0],req.object_pose[1]+175,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
	try:
		if(req.left_to_right == True):
			if(req.tip_pick == True):
				print('Entered tip pick')
				module.pick_and_pour_left_to_right(req.object_pose[0],req.object_pose[1]+175,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
				return PickAndPourResponse(True)
			else:
				print('Entered no tip pick')
				module.pick_and_pour_left_to_right(req.object_pose[0],req.object_pose[1]+135,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
				return PickAndPourResponse(True)
		else:
			if(req.tip_pick == True):
				print('Entered tip pick')
				module.pick_and_pour_right_to_left(req.object_pose[0],req.object_pose[1]+175,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
				return PickAndPourResponse(True)
			else:
				print('Entered no tip pick')
				module.pick_and_pour_right_to_left(req.object_pose[0],req.object_pose[1]+135,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
				return PickAndPourResponse(True)
	except:
		print('Pick and pour failed')
		return PickAndPourResponse(False)
	
def pick_and_pour_server():
	s = rospy.Service('/cartesian_movement_services/PickAndPour',PickAndPour,handle_pick_and_pour)
	print('Ready to execute Pick and Pour')

#Pick server
def handle_pick(req):
	print('Executing pick and place')
	set_mode_cartesian()
	print(req)
	succesful = False
	try:
		if(req.is_vertical == True):
			print('Vertical pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Z axis must be increased by 175mm to avoid the tip of the end effector to crush itself with the table
				grasping_z_axis = req.object_pose[2] + 175
				module.vertical_pick(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5])
			else:
				print('Middle pick')
				#The Z axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_z_axis = req.object_pose[2] + 135
				module.vertical_pick(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5])
			module.stand_up_and_see_vertical()
		else:
			print('Horizontal pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Y axis must be increase	d by 175mm to make the tip of the end effector to be in the same position as the object
				grasping_Y_axis = req.object_pose[1] + 175
				module.horizontal_pick(req.object_pose[0],grasping_Y_axis,req.object_pose[2])
			else:
				print('Middle pick')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_Y_axis = req.object_pose[1] + 135
				module.horizontal_pick(req.object_pose[0],grasping_Y_axis,req.object_pose[2])
			module.stand_up_and_see_horizontal()
		# set_state(0)
		# set_mode(4)
		# time.sleep(2.0)
		set_mode_moveit()
		return PickResponse(True)
	except Exception as e:

		print('Pick and place failed')
		print(e)
		# set_state(0)
		# set_mode(4)
		# time.sleep(2.0)
		set_mode_moveit()
		return PickResponse(False)
	
def pick_server():
	s = rospy.Service('/cartesian_movement_services/Pick',Pick,handle_pick)
	print('Ready to execute Pick')

#Place server
def handle_place(req):
	print('Executing place')
	set_mode_cartesian()
	print(req)
	succesful = False
	try:
		if(req.is_vertical == True):
			print('Vertical pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Z axis must be increased by 175mm to avoid the tip of the end effector to crush itself with the table
				grasping_z_axis = req.destination_pose[2] + 175
				module.vertical_place(req.destination_pose[0],req.destination_pose[1],grasping_z_axis,req.destination_pose[5])
			else:
				print('Middle pick')
				#The Z axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_z_axis = req.destination_pose[2] + 135
				module.vertical_place(req.destination_pose[0],req.destination_pose[1],grasping_z_axis,req.destination_pose[5])
		else:
			print('Horizontal pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Y axis must be increase	d by 175mm to make the tip of the end effector to be in the same position as the object
				grasping_Y_axis = req.destination_pose[1] + 175
				module.horizontal_place(req.destination_pose[0],grasping_Y_axis,req.destination_pose[2])
			else:
				print('Middle pick')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_Y_axis = req.destination_pose[1] + 135
				module.horizontal_place(req.destination_pose[0],grasping_Y_axis,req.destination_pose[2])
		set_mode_moveit()
		return PlaceResponse(True)
	except:
		print('Pick and place failed')
		set_mode_moveit()
		return PlaceResponse(False)
	
def place_server():
	s = rospy.Service('/cartesian_movement_services/Place',Place,handle_place)
	print('Ready to execute Place')

def init_servers():
	move_end_effector_server()
	pick_and_place_server()
	pick_and_pour_server()
	pick_server()
	place_server()

def set_mode_cartesian():
    set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
    set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)	
    set_mode(0)
    set_state(0)
    time.sleep(2.0)

def set_mode_moveit():
    set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
    set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)	
    set_mode(1)
    set_state(0)
    time.sleep(2.0)
 

##########Definition of arm services####################

if __name__ == "__main__":
	rospy.init_node('cartesian_server')
	rospy.set_param('/xarm/wait_for_finish', True) # return after motion service finish
	motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
	# set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
	# set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)

	#Configs for cartesian movement
	# motion_en(8,1)
	# set_mode(0)
	# set_state(0)
	# time.sleep(2.0)

	# starting position for servo_cartesian in Base Coordinate
	

	init_servers()
	
	rospy.spin()

