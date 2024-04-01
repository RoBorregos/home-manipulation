#!/usr/bin/env python3

from __future__ import print_function

from cartesian_movement_services.srv import AddTwoInts,AddTwoIntsResponse
from cartesian_movement_services.srv import PickAndPlaceVertical, PickAndPlaceVerticalResponse
from cartesian_movement_services.srv import TurnEndEffector, TurnEndEffectorResponse
from cartesian_movement_services.srv import *
from xarm_msgs.srv import *

# from cartesian_movement_services.arm_scripts.pouring_srv import adjust_end_effector_yaw

import sys
import time
import rospy
import math as m
import copy

############# Arm's functions ####################3

#Returns the arm with joint movements, which implies more risks of obstacle collitions but ensures the arm returns all of the times its called
def return_to_default_pose_horizontal():
	rospy.wait_for_service('/xarm/move_joint')
	joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
	req = MoveRequest() 
	req.mvvelo = 0.5
	req.mvacc = 7
	req.mvtime = 0
	req.mvradii = 0

	try:
		req.pose = [-1.57,-0.7853,-0.7853,-1.57,0,-3.9253]
		joint_move(req)
	except rospy.ServiceException as e:
		print("Default movement failed: %s"%e)
		return -1
	
#Returns the arm with joint movements to the vertical manipulation pose
def return_to_default_pose_vertical():
	rospy.wait_for_service('/xarm/move_joint')
	joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
	req = MoveRequest() 
	req.mvvelo = 0.5
	req.mvacc = 7
	req.mvtime = 0
	req.mvradii = 0

	try:
		req.pose = [-1.57,-0.7853,-1.309,0,2.09,-2.35]
		joint_move(req)
	except rospy.ServiceException as e:
		print("Default movement failed: %s"%e)
		return -1
	
#Adjust the last joint angle
def adjust_end_effector_yaw(joint6_angle):
	rospy.wait_for_service('/xarm/move_joint')
	joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
	get_angle = rospy.ServiceProxy('/xarm/get_servo_angle', GetFloat32List)
	req = MoveRequest() 
	req.mvvelo = 1
	req.mvacc = 7
	req.mvtime = 0
	req.mvradii = 0
	actual_pose = list(get_angle().datas)
	yaw_angle = abs(m.radians(-135+joint6_angle))
	actual_yaw_angle = actual_pose[5]
	yaw_angle_sign = m.copysign(1,joint6_angle)
	if(actual_yaw_angle<0):
		yaw_transformed = -yaw_angle - m.pi
	else:
		yaw_transformed = yaw_angle
	# if(actual_yaw_angle<0):
	# 	yaw_transformed = -yaw_angle - m.pi
	# else:
	# 	yaw_transformed = yaw_angle
	try:
		req.pose = [actual_pose[0],actual_pose[1],actual_pose[2],actual_pose[3],actual_pose[4],yaw_transformed]
		joint_move(req)
		print(actual_yaw_angle)
	except rospy.ServiceException as e:
		print("Default movement failed: %s"%e)
		return -1

#Stb stands for stabilized
#Stb movement to point and execute grasp with the last given orientation of the end effector
def xarm_move_to_point(x,y,z,actual_pose):
	rospy.wait_for_service('/xarm/move_line')
	estabilized_movement = rospy.ServiceProxy('/xarm/move_line', Move)
	req = MoveRequest()
	req.pose = actual_pose
	req.mvvelo = 80
	req.mvacc = 200
	req.mvtime = 0 
	ret = 0

	#Conserve the last given end effector orientation 
	req.pose[0] = x
	req.pose[1] = y
	req.pose[2] = z

	#WARNING: The robot will move the end effector according to the shortest path to its desired pose
	#so there is no warranty that the movement wont imply exceeding one or multiple joint limits
	#In other words, only move this if you are sure that the end effector angles (at least the joint 6 angle) are between pi and -pi
	# req.pose[3] = pith
	# req.pose[4] = roll
	# req.pose[5] = yaw b


	try:
		estabilized_movement(req)
		return ret

	except rospy.ServiceException as e:
		print("Cartesian movement failed: %s"%e)
		return -1

#Check the values for end effector orientation when pouring
def xarm_move_and_pour(x,y,z,roll,speed,actual_pose):
	rospy.wait_for_service('/xarm/move_line')
	estabilized_movement = rospy.ServiceProxy('/xarm/move_line', Move)
	req = MoveRequest()
	req.pose = actual_pose
	req.mvvelo = speed
	req.mvacc = 200
	req.mvtime = 0 
	ret = 0

	#Conserve the last given end effector orientation 
	req.pose[0] = x
	req.pose[1] = y
	req.pose[2] = z

	#WARNING: The robot will move the end effector according to the shortest path to its desired pose
	#so there is no warranty that the movement wont imply exceeding one or multiple joint limits
	#In other words, only move this if you are sure that the end effector angles (at least the joint 6 angle) are between pi and -pi
	# req.pose[3] = pith
	req.pose[4] = roll
	# req.pose[5] = yaw b


	try:
		estabilized_movement(req)
		return ret

	except rospy.ServiceException as e:
		print("Cartesian movement failed: %s"%e)
		return -1
	
#Activates the gripper and waits until its completely closed (sleep must be hardcoded since no feedback from the gripper is given)
def xarm_grasp(action):
	gripper_action = rospy.ServiceProxy('/xarm/set_digital_out',SetDigitalIO)
	gripper_state = SetDigitalIORequest
	gripper_action(1,action)
	time.sleep(1.75)

#Moves the arm from the default pose for cartesian picks and places to the grasping point 
def move_by_coordinates(x,y,z,actual_pose):
	xarm_move_to_point(x,actual_pose[1],actual_pose[2],actual_pose)
	xarm_move_to_point(x,y,actual_pose[2],actual_pose)
	xarm_move_to_point(x,y,z,actual_pose)

#Moves the arm from the grasping point to its default pose for cartesian picks and places
def move_by_coordinates_reverse(x,y,z,actual_pose):
	xarm_move_to_point(actual_pose[0],actual_pose[1],z,actual_pose)	
	xarm_move_to_point(actual_pose[0],y,z,actual_pose)
	xarm_move_to_point(x,y,z,actual_pose)

#Moves the arm first in Z axis, then in X axis finally Y axis
def move_by_coordinates_ZXY(x,y,z,actual_pose):
	xarm_move_to_point(actual_pose[0],actual_pose[1],z,actual_pose)
	xarm_move_to_point(x,actual_pose[1],z,actual_pose)
	xarm_move_to_point(x,y,z,actual_pose)

#The robot moves to the grasping point, executes grasp and returns to default pose using cartesian movements
def move_grab_and_take(x,y,z,actual_pose):
	xarm_grasp(0)
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates(x,y,z,actual_pose)	
	xarm_grasp(1)
	move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2],actual_pose)

#The robot moves to the grasping point, executes degrasp and returns to default pose cartesian movements
def move_grab_and_place(x,y,z,actual_pose):
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates(x,y,z,actual_pose)	
	xarm_grasp(0)
	move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2],actual_pose)

#The robot moves to the grasping point, executes grasp and returns to default pose using joint movements
def move_grab_and_take_joint_return(x,y,z,actual_pose):
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates(x,y,z,actual_pose)	
	xarm_grasp(1)
	return_to_default_pose_horizontal()

#The robot moves to the grasping point, executes degrasp and returns to default pose joint movements
def move_grab_and_place_joint_return(x,y,z,actual_pose):
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates(x,y,z,actual_pose)	
	xarm_grasp(0)
	return_to_default_pose_horizontal()

#The robot will move according the shortest path to its destination in a SINGLE movement and grasp
def move_to_point_and_grasp(x,y,z,actual_pose):
	xarm_move_to_point(x,y,z,actual_pose)
	xarm_grasp(1)

#The robot will move according the shortest path to its destination in a SINGLE movement and degrasp
def move_to_point_and_degrasp(x,y,z,actual_pose):
	xarm_move_to_point(x,y,z,actual_pose)
	xarm_grasp(0)

#The robot will have an object which will be  considered 'taken' by default, and will pour it into the bowl using its center
#as a reference for the pouring algorithm
def take_and_pour(x_pouring_point,y_pouring_point,z_pouring_point,object_h,bowl_h,bowl_radius,actual_pose):
	#In order to test the algorithm, an object with h=21cm and a bowl with h=8.5cms
	#1 cm offset will be given until a mathematical offset is determined
	security_offset = 1

	actual_pose_ = copy.deepcopy(actual_pose)

	#Make sure the object is on the central point of the bowl before pouring (for debugging purposes, might delete later)
	move_by_coordinates_ZXY(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,actual_pose_)

	#Translate the object to the "left" of the central point of the bowl and begin pouring
	xarm_move_and_pour(x_pouring_point+object_h/2+bowl_radius,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,0.7853,40,actual_pose_)

	#Move the bottle pouring
	xarm_move_and_pour(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,2.35,40,actual_pose_)

	#Return the arm
	xarm_move_and_pour(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,-0.7853,200,actual_pose_)
	
	# move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2],actual_pose_)	

#The robot executes a pick with the actual end effector orientation and pours the container 
def pick_and_pour(object_x,object_y,object_z,pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius):
	print('Entered pick and pour')
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	initial_pose = copy.deepcopy(get_position)
	print('Initial pose taken')
	#The robot initialize its movement from the default cartesian movement pose and grasps the object
	print('Executing grab and take')
	move_grab_and_take(object_x,object_y,object_z,initial_pose)
	print('Grab and take executed')
	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	take_and_pour(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)

	#Return to the initial position
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose)

	#Return the object to its origintal position from the current point (must change to move the robot to its default cartesian pose before putting the object into its original pose)
	move_grab_and_place(object_x,object_y,object_z,initial_pose)

#Goes to the vision pose for horizontal places
def stand_up_and_see_horizontal(actual_pose):
	return_to_default_pose_horizontal()
	xarm_move_to_point(76.2,-260,883,actual_pose)

#Goes to default pose from vision pose
def get_down_and_wait_horizontal(actual_pose):
	xarm_move_to_point(76.2,-260,583,actual_pose)
	return_to_default_pose_horizontal()

#Goes to the vision pose for vertical places
def stand_up_and_see_vertical(actual_pose):
	return_to_default_pose_vertical()
	xarm_move_to_point(0,-171,783,actual_pose)

#Goes to default vertical pose from vision pose
def get_down_and_wait_vertical(actual_pose):
	xarm_move_to_point(0,-171,647.7,actual_pose)
	return_to_default_pose_vertical()

#Vertical pick and place asking for grasping point and object orientation
def vertical_pick_and_place(object_x,object_y,object_z,object_orientation,place_x,place_y,place_z,place_orientation):
	#The arm returns to its default position 
	return_to_default_pose_vertical()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	adjust_end_effector_yaw(object_orientation)

	#The new pose with the modified end effector must be registered for the arm to remember it when moving itself while having the object
	pose_yaw_modified = list(get_position().datas)
	pose_yaw_modified_ = copy.deepcopy(pose_yaw_modified)
	move_grab_and_take(object_x,object_y,object_z,pose_yaw_modified_)

	#From the default position with the last joint moved, solve the arm's movement to place the object in the new orientation
	adjust_end_effector_yaw(place_orientation)
	pose_yaw_modified = list(get_position().datas)
	pose_yaw_modified_ = copy.deepcopy(pose_yaw_modified)
	move_grab_and_place(place_x,place_y,place_z,pose_yaw_modified_)

	#Return the arm
	return_to_default_pose_vertical()

#Horizontal pick and place
def horizontal_pick_and_place(object_x,object_y,object_z,destination_x,destination_y,destination_z):
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_pose)
	move_grab_and_take(object_x,object_y,object_z,initial_pose)
	move_grab_and_place(destination_x,destination_y,destination_z,initial_pose)

############# end of arm's functions #####################


#End effector rotation server
def handle_move_end_effector(req):
	print('Moving end effector')	
	adjust_end_effector_yaw(req.degree)
	return TurnEndEffectorResponse(True)

def move_end_effector_server():
	rospy.init_node('move_end_effector_server')
	s = rospy.Service('/cartesian_movement_services/TurnEndEffector',TurnEndEffector,handle_move_end_effector)
	print('Ready to move end effector')
	rospy.spin()

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
				vertical_pick_and_place(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5],req.destination_pose[0],req.destination_pose[1],grasping_z_axis,req.destination_pose[5])
				return PickAndPlaceResponse(True)
			else:
				print('Middle pick')
				#The Z axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_z_axis = req.object_pose[2] + 135
				vertical_pick_and_place(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5],req.destination_pose[0],req.destination_pose[1],grasping_z_axis,req.destination_pose[5])
				return PickAndPlaceResponse(True)
			#vertical_pick_and_place(req.object_pose[0],req.object_pose[1],grasping_z_axis,req.object_pose[5],req.destiantion_pose[0],req.destination_pose[1],req.destination_pose[2],req.destination_pose[5])
		else:
			print('Horizontal pick and place')
			if(req.tip_pick == True):
				print('Tip pick')
				#The Y axis must be increase	d by 175mm to make the tip of the end effector to be in the same position as the object
				grasping_Y_axis = req.object_pose[1] + 175
				horizontal_pick_and_place(req.object_pose[0],grasping_Y_axis,req.object_pose[2],req.destination_pose[0],grasping_Y_axis,req.destination_pose[2])
				return PickAndPlaceResponse(True)
			else:
				print('Middle pick')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_Y_axis = req.object_pose[1] + 135
				horizontal_pick_and_place(req.object_pose[0],grasping_Y_axis,req.object_pose[2],req.destination_pose[0],grasping_Y_axis,req.destination_pose[2])
				return PickAndPlaceResponse(True)
	except:
		print('Pick and place failed')
		return PickAndPlaceResponse(False)


def pick_and_place_server():
	rospy.init_node('pick_and_place_server')
	s = rospy.Service('/cartesian_movement_services/PickAndPlace',PickAndPlace,handle_pick_and_place)
	print('Ready to execute Pick and Place')
	rospy.spin()

#Pouring server
def handle_pick_and_pour(req):
	print('Executing pick and pour')
	print(req)
	successful_attempt = False
	print('Entered pick and pour handle')
	pick_and_pour(0,-330,380,10,-330,380,21,85,70)
	#pick_and_pour(req.object_pose[0],req.object_pose[1]+175,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
	return PickAndPourResponse(True)
	# try:
	# 	if(req.tip_pick == True):
	# 		print('Entered tip pick')
	# 		pick_and_pour(req.object_pose[0],req.object_pose[1]+175,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
	# 		return PickAndPourResponse(True)
	# 	else:
	# 		print('Entered no tip pick')
	# 		pick_and_pour(req.object_pose[0],req.object_pose[1]+135,req.object_pose[2],req.pouring_point[0],req.pouring_point[1],req.pouring_point[2],req.object_height,req.bowl_height,req.bowl_radius)
	# 		return PickAndPourResponse(True)
	# except:
	# 	print('Pick and pour failed')
	# 	return PickAndPourResponse(False)
	
def pick_and_pour_server():
	rospy.init_node('pick_and_pour_server')
	s = rospy.Service('/cartesian_movement_services/PickAndPour',PickAndPour,handle_pick_and_pour)
	print('Ready to execute Pick and Pour')
	rospy.spin()


if __name__ == "__main__":
	rospy.set_param('/xarm/wait_for_finish', True) # return after motion service finish
	motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
	set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
	set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)

	#Configs for cartesian movement
	motion_en(8,1)
	set_mode(0)
	set_state(0)

	# starting position for servo_cartesian in Base Coordinate
	time.sleep(2.0)

	pick_and_pour_server()
