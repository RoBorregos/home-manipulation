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
	time.sleep(3.0)

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
	return_to_default_pose()

#The robot moves to the grasping point, executes degrasp and returns to default pose joint movements
def move_grab_and_place_joint_return(x,y,z,actual_pose):
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates(x,y,z,actual_pose)	
	xarm_grasp(0)
	return_to_default_pose()

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
def pick_and_pour(object_x,object_y,object_z,pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,actual_pose):
	initial_pose = copy.deepcopy(actual_pose)
	print(initial_pose)
	#The robot initialize its movement from the default cartesian movement pose and grasps the object
	move_grab_and_take(object_x,object_y,object_z,initial_pose)
	print(initial_pose)

	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	take_and_pour(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)
	print(initial_pose)

	#Return to the initial position
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose)
	print(initial_pose)

	#Return the object to its origintal position from the current point (must change to move the robot to its default cartesian pose before putting the object into its original pose)
	move_grab_and_place(object_x,object_y,object_z,initial_pose)
	print(initial_pose)

#Goes to the vision pose for horizontal places
def stand_up_and_see_horizontal(actual_pose):
	return_to_default_pose()
	xarm_move_to_point(76.2,-260,883,actual_pose)

#Goes to default pose from vision pose
def get_down_and_wait(actual_pose):
	xarm_move_to_point(76.2,-260,583,actual_pose)
	return_to_default_pose()

#Main callback
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

	time.sleep(2.0)

	#Predefined values for testing purposes in mm:
	#Bowl Z value is 260mm which is almost the same as the table value
	bowl_height = 85 
	container_height = 210
	bowl_radius = 70

	#################################################
	# actual_pose = list(get_position().datas)
	# move_grab_and_place(220,-300,380,actual_pose)
	# xarm_grasp(0)
	return_to_default_pose()
	actual_pose = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_pose)
	stand_up_and_see_horizontal(initial_pose)
	time.sleep(3)
	get_down_and_wait(initial_pose)
	pick_and_pour(220,-320,350,-220,-320,260,container_height,bowl_height,bowl_radius,initial_pose)
	# move_grab_and_take(220,-320,350,actual_pose)
	#take_and_pour(-220,-360,300,container_height,bowl_height,bowl_radius,actual_pose)
	# move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2],actual_pose)
	# move_grab_and_place(220,-320,380,initial_pose)
	#xarm_move_to_point(-220,-300,260+container_height/2+bowl_height+10,actual_pose)
	############################################

	#Close gripper: xarm_grasp(1)
	#Open gripper: xarm_grasp(0)

	# print('Entering default pose')
	# return_to_default_pose()
	# print('Exiting default pose')

	# if xarmo_move_line_callback(actual_pose) == 0:
	# 	print("execution finished successfully!")
	

if __name__ == "__main__":
	cartesian_movement_callback()
    