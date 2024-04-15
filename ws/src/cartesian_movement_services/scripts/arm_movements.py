#!/usr/bin/env python3
import time
import rospy
from xarm_msgs.srv import *
import copy
import math as m

class arm:

	#######################XARM ESSENTIAL FUNCTIONS#######################

	def __init__(self):
		#Connect to the xarm services and set the parameters
		rospy.wait_for_service('/xarm/move_line')
		rospy.set_param('/xarm/wait_for_finish', True)
		
		self.error_status = 0
		self.gripper_open = 1
		self.mode = 1
		self.state = 0

		#List at least 2 or 3 poses to try to execute the movement
		self.actual_pose = 0
		self.is_vertical = False
	
	#Set cartesian mode for the API to work
	def set_mode_cartesian(self):
		set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
		set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)	
		self.error_status = 0
		self.mode = 0
		self.state = 0
		set_mode(0)
		set_state(0)
		print("Cartesian mode set")
		time.sleep(2.0)

	#Set servo velocities for moveit 
	def set_mode_moveit(self):
		set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
		set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)	
		set_mode(1)
		set_state(0)
		self.mode = 1
		self.state = 0
		print("Moveit mode set")
		time.sleep(2.0)

	#Set the gripper to open
	def set_gripper(self,action):
		rospy.wait_for_service('/xarm/set_digital_out')
		gripper_action = rospy.ServiceProxy('/xarm/set_digital_out',SetDigitalIO)
		#Check if the robot stopped suddenly in the routine to NOT open the gripper in a wrong position
		if(self.error_status == 0):
			if(action == 1):
				self.gripper_open = 1
			else:
				self.gripper_open = 0
			gripper_action(1,action)
			time.sleep(1.75)
		else:
			print("Gripper not moving in faulting state")

	#Returns the arm with joint movements, which implies more risks of obstacle collitions but ensures the arm returns everytime
	def return_to_default_pose_horizontal(self):
		rospy.wait_for_service('/xarm/move_joint')
		joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
		req = MoveRequest() 
		req.mvvelo = 0.5
		req.mvacc = 7
		req.mvtime = 0
		req.mvradii = 0
		try:
			self.set_mode_cartesian()
			self.is_vertical = False
			req.pose = [-1.5707963705062866, -1.0471975803375244, -1.0471975803375244, 0.0, 0.5235987901687622, 0.7853981852531433]
			joint_move(req)
			
		except rospy.ServiceException as e:
			print("Default horizontal movement failed: %s"%e)
			self.error_status = 1
	
	#Returns the arm with joint movements to the vertical manipulation pose
	def return_to_default_pose_vertical(self):
		self.set_mode_cartesian()
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
			print("Trying to move to vertical pose")
			# self.set_mode_cartesian()
			self.is_vertical = True
			req.pose = [-1.57,-0.7853,-1.309,0,2.09,-2.35]
			joint_move(req)
		except rospy.ServiceException as e:
			print("Default movement failed: %s"%e)

	#Move arm joint
	def move_joint(self,joint_number,radians):
		rospy.wait_for_service('/xarm/move_joint')
		joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
		get_angle = rospy.ServiceProxy('/xarm/get_servo_angle', GetFloat32List)
		req = MoveRequest() 
		req.mvvelo = 1
		req.mvacc = 7
		req.mvtime = 0
		req.mvradii = 0
		actual_pose = list(get_angle().datas)
		#If the selected joint is the 6th, the yaw angle is calculated manually according to the EE offset
		if(joint_number == 5):
			yaw_angle = radians-m.radians(45)
			# get shortest path	
			if(actual_pose[5]<0):
				yaw_angle = -yaw_angle - m.pi
			actual_pose[joint_number] = yaw_angle
		else:
			actual_pose[joint_number] = radians
		try:
			req.pose = [actual_pose[0],actual_pose[1],actual_pose[2],actual_pose[3],actual_pose[4],actual_pose[5]]
			joint_move(req)
		except rospy.ServiceException as e:
			print("Joint movement failed: %s"%e)
			self.error_status = 1
	
	#Move arm to a cartesian point
	def xarm_move_to_point(self,x,y,z):
		print('moving to point')
		rospy.wait_for_service('/xarm/move_line')
		estabilized_movement = rospy.ServiceProxy('/xarm/move_line', Move)
		req = MoveRequest()
		print('request made')
		get_pose = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
		actual_pose = list(get_pose().datas)
		velocity = 150 
		req.pose = actual_pose
		req.mvvelo = velocity
		req.mvacc = 200
		req.mvtime = 0 
		ret = 0
		trials = 3
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
		print('initializing trial')
		while(trials > 0):
			try:
				print('trying to move')
				req.mvvelo = velocity
				estabilized_movement(req)
				print(req)
				print('moved')
				self.error_status = 0
				ret = 0
				trials = 0
				return ret

			except rospy.ServiceException as e:
				print("Cartesian movement failed: %s"%e)
				self.error_status = -1
				ret = -1
				trials = trials - 1
				velocity = velocity/2
				return ret
			
	#Move to pour
	def xarm_move_to_pour(self,x,y,z,r):
		print('moving to point')
		rospy.wait_for_service('/xarm/move_line')
		estabilized_movement = rospy.ServiceProxy('/xarm/move_line', Move)
		req = MoveRequest()
		print('request made')
		get_pose = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
		actual_pose = list(get_pose().datas)
		velocity = 50
		req.pose = actual_pose
		req.mvvelo = velocity
		req.mvacc = 200
		req.mvtime = 0 
		ret = 0
		trials = 3
		#Conserve the last given end effector orientation
		
		req.pose[0] = x
		req.pose[1] = y
		req.pose[2] = z
		req.pose[4] = r

		#WARNING: The robot will move the end effector according to the shortest path to its desired pose
		#so there is no warranty that the movement wont imply exceeding one or multiple joint limits
		#In other words, only move this if you are sure that the end effector angles (at least the joint 6 angle) are between pi and -pi
		# req.pose[3] = pith
		# req.pose[4] = roll
		# req.pose[5] = yaw b
		try:
			print('trying to move')
			req.mvvelo = velocity
			estabilized_movement(req)
			print(req)
			print('moved')
			self.error_status = 0
			ret = 0
			trials = 0
			return ret
		
		except rospy.ServiceException as e:
			print("Cartesian movement failed: %s"%e)
			self.error_status = -1
			ret = -1
			trials = trials - 1
			velocity = velocity/2
			set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
			set_mode(0)
			set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
			set_state(0)
			time.sleep(2.0)
			return ret

	#Move arm tool in a cartesian plane
	def move_tool(self,x,y,z):
		print('moving tool')
		rospy.wait_for_service('/xarm/move_line_tool')
		estabilized_movement = rospy.ServiceProxy('/xarm/move_line_tool', Move)
		req = MoveRequest()
		print('request made')
		get_pose = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
		actual_pose = list(get_pose().datas)
		velocity = 100
		req.pose = actual_pose
		req.mvvelo = velocity
		req.mvacc = 200
		req.mvtime = 0
		ret = 0
		#Conserve the last given end effector orientation
		req.pose[0] = x
		req.pose[1] = y
		req.pose[2] = z

		try:
			estabilized_movement(req)
			return ret
		except rospy.ServiceException as e:
			print("Cartesian movement failed: %s"%e)
			self.error_status = -1
			ret = -1
			return ret		
		#######################XARM ESSENTIAL FUNCTIONS#######################
   
	#Move arm to a cartesian point using coordinates
	def move_by_coordinates(self,x,y,z,order,reverse,is_tuple):
		print('entering move by coordinates')
		print(x,y,z)
		get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
		actual_pose = list(get_position().datas)
		actual_pose_ = copy.deepcopy(actual_pose)
		if(is_tuple == False):
			if(order == "XYZ"):
				if(reverse == False):
					self.xarm_move_to_point(x,actual_pose_[1],actual_pose_[2])
					self.xarm_move_to_point(x,y,actual_pose_[2])
					self.xarm_move_to_point(x,y,z)
				else:
					self.xarm_move_to_point(actual_pose_[0],actual_pose_[1],z)
					self.xarm_move_to_point(actual_pose_[0],y,z)
					self.xarm_move_to_point(x,y,z)
			elif(order == "ZXY"):
				if(reverse == False):
					self.xarm_move_to_point(actual_pose_[0],actual_pose_[1],z)
					self.xarm_move_to_point(x,actual_pose_[1],z)
					self.xarm_move_to_point(x,y,z)
				else:
					self.xarm_move_to_point(actual_pose_[0],y,actual_pose_[2])
					self.xarm_move_to_point(x,y,actual_pose_[2])
					self.xarm_move_to_point(x,y,z)
			elif(order == "YZX"):
				if(reverse == False):
					self.xarm_move_to_point(actual_pose_[0],y,actual_pose_[2])
					self.xarm_move_to_point(actual_pose_[0],y,z)
					self.xarm_move_to_point(x,y,z)
				else:
					self.xarm_move_to_point(x,actual_pose_[1],actual_pose_[2])
					self.xarm_move_to_point(x,actual_pose_[1],z)
					self.xarm_move_to_point(x,y,z)
			elif(order == "XZY"):
				if(reverse == False):
					self.xarm_move_to_point(x,actual_pose_[1],actual_pose_[2])
					self.xarm_move_to_point(x,actual_pose_[1],z)
					self.xarm_move_to_point(x,y,z)
				else:
					self.xarm_move_to_point(actual_pose_[0],y,actual_pose_[2])
					self.xarm_move_to_point(actual_pose_[0],y,z)
					self.xarm_move_to_point(x,y,z)
		else:
			if(order == "XYZ"):
				if(reverse == False):
					self.xarm_move_to_point(x,actual_pose_[1],actual_pose_[2])
					self.xarm_move_to_point(x,y,actual_pose_[2])
					self.xarm_move_to_point(x,y,z)
				else:
					self.xarm_move_to_point(actual_pose_[0],actual_pose_[1],z)
					self.xarm_move_to_point(actual_pose_[0],y,z)
					self.xarm_move_to_point(x,y,z)
			
	#Move arm to a pick point
	def pick(self,object_pose,is_vertical,tip_pick):
		print('Entered pick service')
		self.set_gripper(0)
		if(is_vertical == True):
			self.return_to_default_pose_vertical()
			get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
			actual_pose = list(get_position().datas)
			actual_pose_ = copy.deepcopy(actual_pose)
			print('Vertical pick')
			print(object_pose)
			self.move_joint(5,object_pose[5])
			if(tip_pick == True):
				print('Tip pick')
				#The Z axis must be increased by 175mm to avoid the tip of the end effector to crush itself with the table
				grasping_z_axis = object_pose[2] + 175
				self.move_by_coordinates(object_pose[0],object_pose[1],grasping_z_axis,"XYZ",False,False)
				self.set_gripper(1)
				self.move_by_coordinates(actual_pose_[0],actual_pose_[1],actual_pose_[2],"XYZ",True,False)
			else:
				print('Middle pick')
				#The Z axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_z_axis = object_pose[2] + 135
				self.move_by_coordinates(object_pose[0],object_pose[1],grasping_z_axis,"XYZ",False,False)
				self.set_gripper(1)
				self.move_by_coordinates(actual_pose_[0],actual_pose_[1],actual_pose_[2],"XYZ",True,False)
			self.return_to_default_pose_vertical()
		else:
			print('Horizontal pick')
			self.return_to_default_pose_horizontal()
			get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
			actual_pose = list(get_position().datas)
			actual_pose_ = copy.deepcopy(actual_pose)
			print('Changing angle')
			####grasping angle calculation######
			print(object_pose)
			angle = m.atan2(object_pose[1], object_pose[0])
			print(m.degrees(angle))
			self.move_joint(0,angle)
			####################################
			if(tip_pick == True):
				print('Tip pick')
				#The Y axis must be increased by 175mm to make the tip of the end effector to be in the same position as the object
				grasping_Y_axis = object_pose[1] + (m.sin(abs(angle))*175)
				grasping_X_axis = object_pose[0] - (m.cos(angle)*175)
				self.xarm_move_to_point(grasping_X_axis,grasping_Y_axis,object_pose[2])
				self.set_gripper(1)
			else:
				print('Middle pick')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_Y_axis = object_pose[1] + (m.sin(abs(angle))*135)
				grasping_X_axis = object_pose[0] - (m.cos(angle)*135)
				self.xarm_move_to_point(grasping_X_axis,grasping_Y_axis,object_pose[2])
				self.set_gripper(1)
			self.return_to_default_pose_horizontal()
  
	#Move arm to a pick point
	def place(self,object_pose,is_vertical,tip_pick):
		print('Entered place service')
		if(is_vertical == True):
			self.return_to_default_pose_vertical()
			get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
			actual_pose = list(get_position().datas)
			actual_pose_ = copy.deepcopy(actual_pose)
			print('Vertical place')
			print(object_pose)
			self.move_joint(5,object_pose[5])
			if(tip_pick == True):
				print('Tip place')
				#The Z axis must be increased by 175mm to avoid the tip of the end effector to crush itself with the table
				grasping_z_axis = object_pose[2] + 175
				self.move_by_coordinates(object_pose[0],object_pose[1],grasping_z_axis,"XYZ",False,False)
				self.set_gripper(0)
				self.move_by_coordinates(actual_pose_[0],actual_pose_[1],actual_pose_[2],"XYZ",True,False)
			else:
				print('Middle place')
				#The Z axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_z_axis = object_pose[2] + 135
				self.move_by_coordinates(object_pose[0],object_pose[1],grasping_z_axis,"XYZ",False,False)
				self.set_gripper(0)
				self.move_by_coordinates(actual_pose_[0],actual_pose_[1],actual_pose_[2],"XYZ",True,False)
			self.return_to_default_pose_vertical()
		else:
			print('Horizontal place')
			self.return_to_default_pose_horizontal()
			get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
			actual_pose = list(get_position().datas)
			actual_pose_ = copy.deepcopy(actual_pose)
			print('Changing angle')
			####grasping angle calculation######
			print(object_pose)
			angle = m.atan2(object_pose[1], object_pose[0])
			print(m.degrees(angle))
			self.move_joint(0,angle)
			####################################
			if(tip_pick == True):
				print('Tip place')
				#The Y axis must be increased by 175mm to make the tip of the end effector to be in the same position as the object
				grasping_Y_axis = object_pose[1] + (m.sin(abs(angle))*175)
				grasping_X_axis = object_pose[0] - (m.cos(angle)*175)
				self.xarm_move_to_point(grasping_X_axis,grasping_Y_axis,object_pose[2])
				self.set_gripper(0)
			else:
				print('Middle place')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				grasping_Y_axis = object_pose[1] + (m.sin(abs(angle))*135)
				grasping_X_axis = object_pose[0] - (m.cos(angle)*135)
				self.xarm_move_to_point(grasping_X_axis,grasping_Y_axis,object_pose[2])
				self.set_gripper(0)
			self.return_to_default_pose_horizontal()
		
	def pour(self,destination_pose,grasp_h,left_to_pick,bowl_radius,bowl_height,left_to_right,tip_pick):
		self.return_to_default_pose_horizontal()
		absolute_height = destination_pose[2] + bowl_height + grasp_h
		offset = 20 #Change if pour is executing too far from the bowl
		print('Pouring')
		#self.move_by_coordinates(destination_pose[0],destination_pose[1],absolute_height,"ZXY",False,False)
		if(left_to_right):
			print('Pouring from left to right')
			if(tip_pick == True):
				print('Tip pour')
				#The Y axis must be increased by 175mm to make the tip of the end effector to be in the same position as the object
				absolute_traslation = destination_pose[0] + bowl_radius + left_to_pick - offset
				self.move_by_coordinates(absolute_traslation,destination_pose[1]+175,absolute_height,"XZY",False,False)
				self.xarm_move_to_pour(destination_pose[0],destination_pose[1]+175,absolute_height,2.3)
			else:
				print('Middle pour')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				absolute_traslation = destination_pose[0] + bowl_radius + left_to_pick - offset
				self.move_by_coordinates(absolute_traslation,destination_pose[1]+135,absolute_height,"XZY",False,False)
				self.xarm_move_to_pour(destination_pose[0],destination_pose[1]+135,absolute_height,2.3)
		else:
			print('Pouring from right to left')
			if(tip_pick == True):
				print('Tip pour')
				#The Y axis must be increased by 175mm to make the tip of the end effector to be in the same position as the object
				absolute_traslation = destination_pose[0] - bowl_radius - left_to_pick + offset
				self.move_by_coordinates(absolute_traslation,destination_pose[1]+175,absolute_height,"XZY",False,False)
				self.xarm_move_to_pour(destination_pose[0],destination_pose[1]+175,absolute_height,-3.59)
			else:
				print('Middle pour')
				#The Y axis must be increased by 135mm to grasp the object in the middle of the gripper intsead of the end of it
				absolute_traslation = destination_pose[0] - bowl_radius - left_to_pick + offset
				print("Absolute traslation: ")
				self.move_by_coordinates(absolute_traslation,destination_pose[1]+135,absolute_height,"XZY",False,False)
				self.xarm_move_to_pour(destination_pose[0],destination_pose[1]+135,absolute_height,-3.59)
		self.return_to_default_pose_horizontal()
					
#######################XARM MOVEMENT FUNCTIONS#######################

	
