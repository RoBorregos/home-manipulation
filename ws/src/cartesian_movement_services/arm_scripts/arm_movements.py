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
		rospy.init_node('xarm_stabilized_movement',anonymous=False)
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
		print("Setting mode to cartesian")
		set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
		set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)	
		self.error_status = 0
		self.mode = 0
		self.state = 0
		set_mode(0)
		set_state(0)
		time.sleep(2.0)

	#Set servo velocities for moveit 
	def set_mode_moveit(self):
		print("Setting mode to moveit")
		set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
		set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)	
		print("Setting mode to 1")
		set_mode(1)
		set_state(0)
		self.mode = 1
		self.state = 0
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
			# self.set_mode_cartesian()
			self.is_vertical = False
			req.pose = [-1.5707963705062866, -1.0471975803375244, -1.0471975803375244, 0.0, 0.5235987901687622, 0.7853981852531433]
			joint_move(req)
			
		except rospy.ServiceException as e:
			print("Default horizontal movement failed: %s"%e)
			self.error_status = 1
	
	#Returns the arm with joint movements to the vertical manipulation pose
	def return_to_default_pose_vertical(self):
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
			yaw_angle = m.radians(m.degrees(joint_number)-45)
			# get shortest path	
			actual_yaw_angle = actual_pose[5]
			if(actual_yaw_angle<0):
				yaw_angle = -yaw_angle - m.pi
			radians = yaw_angle
		
		actual_pose[joint_number] = radians
		
		try:
			req.pose = actual_pose
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

		get_pose = rospy.ServiceProxy('/xarm/get_position', GetFloat32List)
		actual_pose = list(get_pose().datas)

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
			self.error_status = 0
			return self.error_status

		except rospy.ServiceException as e:
			print("Cartesian movement failed: %s"%e)
			self.error_status = -1
			return self.error_status
		
		#######################XARM ESSENTIAL FUNCTIONS#######################
   
	#######################XARM MOVEMENT FUNCTIONS#######################

