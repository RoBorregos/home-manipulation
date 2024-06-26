#!/usr/bin/env python3
import sys
import time
import rospy
from xarm_msgs.srv import *
import copy
import math as m

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
	yaw_angle = m.radians(45+joint6_angle)
	yaw_angle_fixed = m.trunc(actual_pose[5]/m.radians(90))
	if(yaw_angle_fixed<0):
		yaw_transformed = -yaw_angle
	else:
		yaw_transformed = yaw_angle
	try:
		req.pose = [actual_pose[0],actual_pose[1],actual_pose[2],actual_pose[3],actual_pose[4],yaw_transformed]
		joint_move(req)
		print(yaw_transformed)
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
def vertical_pick_and_place(object_x,object_y,object_z,object_h,object_orientation,place_x,place_y,place_z,place_orientation):
	#The arm returns to its default position 
	return_to_default_pose_vertical()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	adjust_end_effector_yaw(object_orientation)

	#The new pose with the modified end effector must be registered for the arm to remember it when moving itself while having the object
	pose_yaw_modified = list(get_position().datas)
	pose_yaw_modified_ = copy.deepcopy(pose_yaw_modified)
	move_grab_and_take(object_x,object_y,object_z+object_h,pose_yaw_modified_)

	#From the default position with the last joint moved, solve the arm's movement to place the object in the new orientation
	adjust_end_effector_yaw(place_orientation)
	pose_yaw_modified = list(get_position().datas)
	pose_yaw_modified_ = copy.deepcopy(pose_yaw_modified)
	move_grab_and_place(place_x,place_y,place_z+object_h,pose_yaw_modified_)

	#Return the arm
	return_to_default_pose_vertical()

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
	#Table height from the xarm's base perspective when the gripper is grasping vertically 
	table_heigh_vertical_pick = 420
	bowl_height = 85 
	container_height = 210
	bowl_radius = 70
	cereal_height = 120
	spoon_height = 18.5
	spoon_orientation = 45
	cereal_orientation = 45
	yaw_angle_cereal = m.radians(135-cereal_orientation)
	final_orientation = 0

	#Distance between the end effector and grasping point of the gripper (in mms)
	tip_point_offset = 175

	#Distance between the end effector and grasping-by-hug point of the gripper (in mms)
	hug_point_offset = 130

	#Position 1 (Vertical):
	#220,-450,420
	#Position 1 (Horizontal):
	#220,-380,380
	


	#################################################
	actual_pose = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_pose)
	#vertical_pick_and_place(220,-450,420,spoon_height,spoon_orientation,-220,-450,420,final_orientation)
	return_to_default_pose_vertical()
	adjust_end_effector_yaw(90)
	# actual_pose = list(get_position().datas)
	# move_grab_and_place(220,-300,380,actual_pose)
	# xarm_grasp(0)
	#return_to_default_pose_vertical()
	# stand_up_and_see_horizontal(initial_pose)
	# time.sleep(3)
	# get_down_and_wait_horizontal(initial_pose)
	# return_to_default_pose_vertical()
	# stand_up_and_see_vertical(initial_pose)
	# time.sleep(2)
	# get_down_and_wait_vertical(initial_pose)
	# actual_pose = list(get_position().datas)
	# initial_pose = copy.deepcopy(actual_pose)
	# adjust_end_effector_yaw(yaw_angle)
	# pose_with_changed_end_effector = list(get_position().datas)
	# pose_with_changed_end_effector_ = copy.deepcopy(pose_with_changed_end_effector)
	# move_grab_and_take(220,-450,420+20,pose_with_changed_end_effector_)
	# move_grab_and_place(-220,-450,420+20,initial_pose)
	#adjust_end_effector_yaw(yaw_angle)
	#pick_and_pour(0,-320,350,-220,-320,260,container_height,bowl_height,bowl_radius,initial_pose)
	#pick_and_pour(220,-320,350,-220,-320,260,cereal_height,bowl_height,bowl_radius,initial_pose)
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
    