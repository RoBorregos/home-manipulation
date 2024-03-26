#!/usr/bin/env python3
import sys
import time
import rospy
from xarm_msgs.srv import *
import copy

#Returns the arm with joint movements, which implies more risks of obstacle collitions but ensures the arm returns all of the times its called
def return_to_default_pose():
	rospy.wait_for_service('/xarm/move_joint')
	joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
	req = MoveRequest() 
	req.mvvelo = 0.35
	req.mvacc = 7
	req.mvtime = 0
	req.mvradii = 0

	try:
		req.pose = [-1.57,-1.309,-0.261799,-3.1415,0,0.785398]
		joint_move(req)
	except rospy.ServiceException as e:
		print("Default movement failed: %s"%e)
		return -1

#Stb stands for stabilized
#Stb movement to point and execute grasp with the last given orientation of the end effector
def xarm_move_to_point(x,y,z,start_pose):
	rospy.wait_for_service('/xarm/move_line')
	stb_mov = rospy.ServiceProxy('/xarm/move_line',Move)
	estabilized_movement = rospy.ServiceProxy('/xarm/move_line', Move)
	req = MoveRequest()
	req.pose = start_pose
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
	# req.pose[5] = yaw


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
	time.sleep(3.0)

#Moves the arm from the default pose for cartesian picks and places to the grasping point 
def move_by_coordinates(x,y,z,start_pose):
	xarm_move_to_point(x,start_pose[1],start_pose[2],start_pose)
	xarm_move_to_point(x,y,start_pose[2],start_pose)
	xarm_move_to_point(x,y,z,start_pose)

#Moves the arm from the grasping point to its default pose for cartesian picks and places
def move_by_coordinates_reverse(x,y,z,start_pose):
	xarm_move_to_point(start_pose[0],start_pose[1],z,start_pose)	
	xarm_move_to_point(start_pose[0],y,z,start_pose)
	xarm_move_to_point(x,y,z,start_pose)

#The robot moves to the grasping point, executes grasp and returns to default pose using cartesian movements
def move_grab_and_take(x,y,z,start_pose):
	start_pose_ = copy.deepcopy(start_pose)
	move_by_coordinates(x,y,z,start_pose)	
	xarm_grasp(1)
	move_by_coordinates_reverse(start_pose_[0],start_pose_[1],start_pose_[2],start_pose)

#The robot moves to the grasping point, executes degrasp and returns to default pose cartesian movements
def move_grab_and_place(x,y,z,start_pose):
	start_pose_ = copy.deepcopy(start_pose)
	move_by_coordinates(x,y,z,start_pose)	
	xarm_grasp(0)
	move_by_coordinates_reverse(start_pose_[0],start_pose_[1],start_pose_[2],start_pose)

#The robot moves to the grasping point, executes grasp and returns to default pose using joint movements
def move_grab_and_take_joint_return(x,y,z,start_pose):
	start_pose_ = copy.deepcopy(start_pose)
	move_by_coordinates(x,y,z,start_pose)	
	xarm_grasp(1)
	return_to_default_pose()

#The robot moves to the grasping point, executes degrasp and returns to default pose joint movements
def move_grab_and_place_joint_return(x,y,z,start_pose):
	start_pose_ = copy.deepcopy(start_pose)
	move_by_coordinates(x,y,z,start_pose)	
	xarm_grasp(0)
	return_to_default_pose()

#The robot will move according the shortest path to its destination in a SINGLE movement and grasp
def move_to_point_and_grasp(x,y,z,start_pose):
	xarm_move_to_point(x,y,z,start_pose)
	xarm_grasp(1)

#The robot will move according the shortest path to its destination in a SINGLE movement and degrasp
def move_to_point_and_degrasp(x,y,z,start_pose):
	xarm_move_to_point(x,y,z,start_pose)
	xarm_grasp(0)


def cartesian_movement_callback():
	rospy.init_node('xarm_stabilized_movement',anonymous=False)
	rospy.wait_for_service('/xarm/move_line')
	rospy.set_param('/xarm/wait_for_finish', True) # return after motion service finish
	

	motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
	set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
	set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)


	#Configs for cartesian movement
	set_mode(0)
	set_state(0)

	# starting position for servo_cartesian in Base Coordinate
	start_pose = list(get_position().datas)

	time.sleep(2.0)
	return_to_default_pose()
	move_grab_and_take(-220,-300,360,start_pose)
	move_grab_and_place(220,-300,360,start_pose)
	return_to_default_pose()

	#Close gripper: xarm_grasp(1)
	#Open gripper: xarm_grasp(0)

	# print('Entering default pose')
	# return_to_default_pose()
	# print('Exiting default pose')

	# if xarmo_move_line_callback(start_pose) == 0:
	# 	print("execution finished successfully!")
	

if __name__ == "__main__":
	cartesian_movement_callback()
    