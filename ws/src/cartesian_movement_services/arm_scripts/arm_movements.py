#!/usr/bin/env python3
import time
import rospy
from xarm_msgs.srv import *
import copy
import math as m

############# Arm's functions ####################3

global error_status

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
		req.pose = [-1.5707963705062866, -1.0471975803375244, -1.0471975803375244, 0.0, 0.5235987901687622, 0.7853981852531433]
		joint_move(req)
	except rospy.ServiceException as e:
		print("Default movement failed: %s"%e)
		error_status = -1
		return error_status
	
#Returns the arm with joint movements to the vertical manipulation pose
def return_to_default_pose_vertical():
	print("Waiting for move_joint")
	# rospy.wait_for_service('/xarm/move_joint')
	print("move_joint service found")
	joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
	req = MoveRequest() 
	req.mvvelo = 0.5
	req.mvacc = 7
	req.mvtime = 0
	req.mvradii = 0

	try:
		print("Moving to vertical pose")
		req.pose = [-1.57,-0.7853,-1.309,0,2.09,-2.35]
		joint_move(req)
	except rospy.ServiceException as e:
		print("Default movement failed: %s"%e)
		error_status = -1
		return error_status

	print("Returned to default vertical pose")
	
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
	yaw_angle = m.radians(m.degrees(joint6_angle)-45)
	actual_yaw_angle = actual_pose[5]
	if(actual_yaw_angle<0):
		yaw_angle = -yaw_angle - m.pi
	print(f"-------------------------\n Trying to move to {yaw_angle} \n-------------------------")
	try:
		req.pose = [actual_pose[0],actual_pose[1],actual_pose[2],actual_pose[3],actual_pose[4],yaw_angle]
		joint_move(req)
		error_status = 0
		# print(actual_yaw_angle)
	except rospy.ServiceException as e:
		print("Yaw movement failed: %s"%e)
		error_status = -1
		return error_status

#Stb stands for stabilized
#Stb movement to point and execute grasp with the last given orientation of the end effector
def xarm_move_to_point(x,y,z,actual_pose):
	print('moving to point')
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
		error_status = -1
		return error_status

#Function still in development
def xarm_move_end_effector(roll,pitch,yaw,actual_pose):
	print('moving to point')
	rospy.wait_for_service('/xarm/move_line')
	estabilized_movement = rospy.ServiceProxy('/xarm/move_line', Move)
	req = MoveRequest()
	req.pose = actual_pose
	req.mvvelo = 80
	req.mvacc = 200
	req.mvtime = 0 
	ret = 0

	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
 
	#Conserve the last given end effector orientation 
	req.pose[0] = actual_pose_[0]
	req.pose[1] = actual_pose_[1]
	req.pose[2] = actual_pose_[2]

	#WARNING: The robot will move the end effector according to the shortest path to its desired pose
	#so there is no warranty that the movement wont imply exceeding one or multiple joint limits
	#In other words, only move this if you are sure that the end effector angles (at least the joint 6 angle) are between pi and -pi
	req.pose[3] = pitch
	req.pose[4] = roll
	req.pose[5] = yaw


	try:
		estabilized_movement(req)
		return ret

	except rospy.ServiceException as e:
		print("Cartesian movement failed: %s"%e)
		error_status = -1
		return error_status

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
		error_status = -1
		return error_status
	
#Activates the gripper and waits until its completely closed (sleep must be hardcoded since no feedback from the gripper is given)
def xarm_grasp(action):
	rospy.wait_for_service('/xarm/set_digital_out')
	gripper_action = rospy.ServiceProxy('/xarm/set_digital_out',SetDigitalIO)
	gripper_action(1,action)
	time.sleep(1.75)

#Moves the arm from the default pose for cartesian picks and places to the grasping point 
def move_by_coordinates(x,y,z,actual_pose):
	print('entering move by coordinates')
	print(x,y,z)
	xarm_move_to_point(x,actual_pose[1],actual_pose[2],actual_pose)
	xarm_move_to_point(x,y,actual_pose[2],actual_pose)
	xarm_move_to_point(x,y,z,actual_pose)

#Moves the arm from the grasping point to its default pose for cartesian picks and places
def move_by_coordinates_reverse(x,y,z):
	print('entering move by coordinates reverse')
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
	xarm_move_to_point(actual_pose_[0],actual_pose_[1],z,actual_pose_)	
	xarm_move_to_point(actual_pose_[0],y,z,actual_pose_)
	xarm_move_to_point(x,y,z,actual_pose_)

#Moves the arm from the grasping point to its default pose for cartesian picks and places
def move_by_coordinates_reverse_ZXY(x,y,z):
	print('entering move by coordinates reverse ZXY')
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
	xarm_move_to_point(actual_pose_[0],y,actual_pose[2],actual_pose_)	
	xarm_move_to_point(actual_pose_[0],y,z,actual_pose_)
	xarm_move_to_point(x,y,z,actual_pose_)

#Moves the arm first in Z axis, then in X axis finally Y axis
def move_by_coordinates_ZXY(x,y,z,actual_pose):
	print('entering move by coordinates ZXY')
	xarm_move_to_point(actual_pose[0],actual_pose[1],z,actual_pose)
	xarm_move_to_point(x,actual_pose[1],z,actual_pose)
	xarm_move_to_point(x,y,z,actual_pose)

#The robot moves to the grasping point, executes grasp and returns to default pose using cartesian movements
def move_grab_and_take(x,y,z,actual_pose):
	xarm_grasp(0)
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose2 = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose2)
	move_by_coordinates(x,y,z,actual_pose2)	
	xarm_grasp(1)
	move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2])

#The robot moves to the grasping point, executes degrasp and returns to default pose cartesian movements
def move_grab_and_place(x,y,z,actual_pose2):
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates(x,y,z,actual_pose)	
	xarm_grasp(0)
	move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2])

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
	print('Entering take and pour')
	security_offset = 10

	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)

	#Make sure the object is on the central point of the bowl before pouring (for debugging purposes, might delete later)
	print('Moving to bowl XYZ')
	move_by_coordinates_ZXY(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,actual_pose_)

	#Translate the object to the "left" of the central point of the bowl and begin pouring
	print('Moving to the left of the bowl')
	xarm_move_and_pour(x_pouring_point+object_h/2+bowl_radius,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,0.7853,40,actual_pose_)

	#Move the bottle while pouring
	print('Moving the bottle while pouring')
	xarm_move_and_pour(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,2.35,40,actual_pose_)

	#Return the arm
	print('Returning the object to its original position')
	xarm_move_and_pour(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,-0.7853,200,actual_pose_)
	
	# move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2],actual_pose_)	

#The robot will have an object which will be  considered 'taken' by default, and will pour it into the bowl using its center
#as a reference for the pouring algorithm
def take_and_pour_right_to_left(x_pouring_point,y_pouring_point,z_pouring_point,object_h,bowl_h,bowl_radius,actual_pose):
	#In order to test the algorithm, an object with h=21cm and a bowl with h=8.5cms
	#1 cm offset will be given until a mathematical offset is determined
	print('Entering take and pour')
	security_offset = 10

	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)

	#Make sure the object is on the central point of the bowl before pouring (for debugging purposes, might delete later)
	print('Moving to bowl XYZ')
	move_by_coordinates_ZXY(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,actual_pose_)

	#Translate the object to the "left" of the central point of the bowl and begin pouring
	print('Moving to the left of the bowl')
	xarm_move_and_pour(x_pouring_point-object_h/2-bowl_radius,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,-2.35,40,actual_pose_)

	#Move the bottle while pouring
	print('Moving the bottle while pouring')
	xarm_move_and_pour(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,-3.92,40,actual_pose_)

	#Return the arm
	print('Returning the object to its original position')
	xarm_move_and_pour(x_pouring_point,y_pouring_point,z_pouring_point+object_h/2+bowl_h+security_offset,-0.7853,200,actual_pose_)
	
	# move_by_coordinates_reverse(actual_pose_[0],actual_pose_[1],actual_pose_[2],actual_pose_)	

#The robot executes a pick with the actual end effector orientation and pours the container 
def pick_and_pour(object_x,object_y,object_z,pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius):
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_position = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_position)

	#The robot initialize its movement from the default cartesian movement pose and grasps the object
	print('Entering move grab and take')
	move_grab_and_take(object_x,object_y,object_z,initial_pose)

	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	print('Entering take and pour')
	take_and_pour(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)

	#Return to the initial position
	print('Returning to initial position')
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2])

	#Return the object to its origintal position from the current point (must change to move the robot to its default cartesian pose before putting the object into its original pose)
	print('Returning the object to its original position')
	move_grab_and_place(object_x,object_y,object_z,initial_pose)
	print('After returning the object to its original position')

#The robot executes a pick with the actual end effector orientation and pours the container 
def pour_left_to_right(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius):
	print('Pouring left to right')	
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_position = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_position)
	print('got positions')
	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	print('Entering take and pour')
	take_and_pour(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)

	#Return to the initial position
	print('Returning to initial position')
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2])

#The robot executes a pick with the actual end effector orientation and pours the container 
def pick_and_pour_left_to_right(object_x,object_y,object_z,pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius):
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_position = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_position)

	#The robot initialize its movement from the default cartesian movement pose and grasps the object
	print('Entering move grab and take')
	move_grab_and_take(object_x,object_y,object_z,initial_pose)

	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	print('Entering take and pour')
	take_and_pour(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)

	#Return to the initial position
	print('Returning to initial position')
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2])

	#Return the object to its origintal position from the current point (must change to move the robot to its default cartesian pose before putting the object into its original pose)
	print('Returning the object to its original position')
	move_grab_and_place(object_x,object_y,object_z,initial_pose)
	print('After returning the object to its original position')

#The robot executes a pick with the actual end effector orientation and pours the container 
def pick_and_pour_right_to_left(object_x,object_y,object_z,pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius):
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_position = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_position)

	#The robot initialize its movement from the default cartesian movement pose and grasps the object
	print('Entering move grab and take')
	move_grab_and_take(object_x,object_y,object_z,initial_pose)

	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	print('Entering take and pour')
	take_and_pour_right_to_left(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)

	#Return to the initial position
	print('Returning to initial position')
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2])

	#Return the object to its origintal position from the current point (must change to move the robot to its default cartesian pose before putting the object into its original pose)
	print('Returning the object to its original position')
	move_grab_and_place(object_x,object_y,object_z,initial_pose)
	print('After returning the object to its original position')

#The robot executes a pick with the actual end effector orientation and pours the container 
def pour_right_to_left(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius):
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_position = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_position)

	#Pouring point in Z axis is assumed to be the table's height, though it can be changed for other tasks/scenarios
	print('Entering take and pour')
	take_and_pour_right_to_left(pouring_point_x,pouring_point_y,pouring_point_z,object_height,bowl_height,bowl_radius,initial_pose)

	#Return to the initial position
	print('Returning to initial position')
	move_by_coordinates_reverse(initial_pose[0],initial_pose[1],initial_pose[2])
	
#Goes to the vision pose for horizontal places
def stand_up_and_see_horizontal():
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	xarm_move_to_point(0,-35.3,787.6,actual_pose)

#Goes to default pose from vision pose
def get_down_and_wait_horizontal(actual_pose):
	xarm_move_to_point(76.2,-260,583,actual_pose)
	return_to_default_pose_horizontal()

#Goes to the vision pose for vertical places
def stand_up_and_see_vertical():
	return_to_default_pose_vertical()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	xarm_move_to_point(0,-171,783,actual_pose)

#Goes to default vertical pose from vision pose
def get_down_and_wait_vertical(actual_pose):
	xarm_move_to_point(0,-171,647.7,actual_pose)
	return_to_default_pose_vertical()

#Vertical pick
def vertical_pick(object_x,object_y,object_z,object_orientation):
	#The arm returns to its default position 
	print("Returning to default vertical")
	return_to_default_pose_vertical()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	adjust_end_effector_yaw(object_orientation)

	# The new pose with the modified end effector must be registered for the arm to remember it when moving itself while having the object
	pose_yaw_modified = list(get_position().datas)
	pose_yaw_modified_ = copy.deepcopy(pose_yaw_modified)
	move_grab_and_take(object_x,object_y,object_z,pose_yaw_modified_)

#Vertical place
def vertical_place(place_x,place_y,place_z,place_orientation):
	#The arm returns to its default position 
	return_to_default_pose_vertical()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	adjust_end_effector_yaw(place_orientation)

	#The new pose with the modified end effector must be registered for the arm to remember it when moving itself while having the object
	pose_yaw_modified = list(get_position().datas)
	pose_yaw_modified_ = copy.deepcopy(pose_yaw_modified)
	move_grab_and_place(place_x,place_y,place_z,pose_yaw_modified_)

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

#Horizontal pick
def horizontal_pick(object_x,object_y,object_z):
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_pose)
	move_grab_and_take(object_x,object_y,object_z,initial_pose)

#Horizontal place
def horizontal_place(place_x,place_y,place_z):
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_pose)
	move_grab_and_place(place_x,place_y,place_z,initial_pose)

#Horizontal place
def horizontal_place_shelf(place_x,place_y,place_z):
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates_ZXY(place_x,place_y,place_z,actual_pose)	
	xarm_grasp(0)
	move_by_coordinates_reverse_ZXY(actual_pose_[0],actual_pose_[1],actual_pose_[2])

#Vertical shelf place
def vertical_place_shelf(place_x,place_y,place_z):
	return_to_default_pose_vertical()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	actual_pose_ = copy.deepcopy(actual_pose)
	move_by_coordinates_ZXY(place_x,place_y,place_z,actual_pose)	
	xarm_grasp(0)
	move_by_coordinates_reverse_ZXY(actual_pose_[0],actual_pose_[1],actual_pose_[2])

#Horizontal pick and place
def horizontal_pick_and_place(object_x,object_y,object_z,destination_x,destination_y,destination_z):
	return_to_default_pose_horizontal()
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	actual_pose = list(get_position().datas)
	initial_pose = copy.deepcopy(actual_pose)
	move_grab_and_take(object_x,object_y,object_z,initial_pose)
	move_grab_and_place(destination_x,destination_y,destination_z,initial_pose)

############# end of arm's functions #####################
	
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
	motion_en(8,1)
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

if __name__ == "__main__":
	cartesian_movement_callback()
    