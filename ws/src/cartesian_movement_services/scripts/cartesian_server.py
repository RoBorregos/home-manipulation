#!/usr/bin/env python3

from __future__ import print_function
from frida_manipulation_interfaces.srv import *
from cartesian_movement_services.srv import *
from xarm_msgs.srv import *
import rospkg
import importlib
import sys
import time
import rospy
import math as m
import copy
from arm_movements import arm
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

##########Definition of arm services####################
class ArmServices:
	def __init__(self):
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)

		self.pick_server()
		self.place_server()
		self.pour_server()
		self.move_joint_server()
		self.move_xyz_server()

	def pick_server(self):
		s = rospy.Service('/cartesian_movement_services/Pick',Pick,self.handle_pick)
		print('Ready to execute Pick')

	def handle_pick(self ,req):
		print("Picking object")
		xarm.set_mode_cartesian()
		#vertical picks considers only X,Y,Z and yaw
		if not req.consider_angle:
			xarm.pick([req.object_pose[0],req.object_pose[1],req.object_pose[2],req.object_pose[3],req.object_pose[4],req.object_pose[5]],req.is_vertical,req.tip_pick)
		else:
			xarm.move_pose([req.object_pose[0],req.object_pose[1],req.object_pose[2],req.object_pose[3],req.object_pose[4],req.object_pose[5]])
		xarm.set_mode_moveit()
		return PickResponse(True)

	def place_server(self ):
		s = rospy.Service('/cartesian_movement_services/Place',Place,self.handle_place)
		print('Ready to execute Place')

	def handle_place(self ,req):
		print("Placing object")
		xarm.set_mode_cartesian()
		xarm.place([req.destination_pose[0],req.destination_pose[1],req.destination_pose[2],req.destination_pose[3],req.destination_pose[4],req.destination_pose[5]],req.is_vertical,req.tip_pick)
		xarm.set_mode_moveit()
		return PlaceResponse(True)

	def pour_server(self ):
		s = rospy.Service('/cartesian_movement_services/Pour',Pour,self.handle_pour)
		print('Ready to execute Pour')

	def handle_pour(self ,req):
		print("Pouring object")
		xarm.set_mode_cartesian()
		xarm.pour([req.pouring_point[0],req.pouring_point[1],req.pouring_point[2]],req.bowl_height,req.bowl_radius,req.object_height,req.grasp_height,req.left_to_right,req.tip_pick)
		xarm.set_mode_moveit()
		return PourResponse(True)

	def move_joint_server(self ):
		s = rospy.Service('/cartesian_movement_services/MoveJoint',MoveJoint,self.handle_move_joint)
		print('Ready to execute MoveJoint')

	def handle_move_joint(self ,req):
		print("Moving joints")
		xarm.set_mode_cartesian()
		xarm.move_joint(req.joint_number, m.radians(req.degree))
		xarm.set_mode_moveit()
		return MoveJointResponse(True)

	def move_xyz_server(self ):
		s = rospy.Service('/cartesian_movement_services/MovePose',MovePose,self.handle_move_xyz)
		print('Ready to execute MovePose')

	def handle_move_xyz(self ,req):
		try:
			self.tfBuffer.lookup_transform('base_footprint','BaseBrazo',rospy.Time())
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logerr("TF lookup failed with error: %s", e)
			return MovePoseResponse(False)
		
		scale_factor = 1000
		
		actual_pose = xarm.get_current_pose()
		xarm_pose = PoseStamped()
		xarm_pose.header.frame_id = 'BaseBrazo'
		print(actual_pose)

		xarm_pose.pose.position.x = actual_pose[0] / scale_factor
		xarm_pose.pose.position.y = actual_pose[1] / scale_factor
		xarm_pose.pose.position.z = actual_pose[2] / scale_factor
		xarm_pose.pose.orientation.x = actual_pose[0]
		xarm_pose.pose.orientation.y = actual_pose[0]
		xarm_pose.pose.orientation.z = actual_pose[0]
		xarm_pose.pose.orientation.w = actual_pose[0]


		try:
			xarm_pose = self.tfBuffer.transform(xarm_pose,'base_footprint')
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logerr("TF transform failed with error: %s", e)
			return MovePoseResponse(False)

		print(req.target_pose.x,req.target_pose.y,req.target_pose.z)
		xarm_pose.pose.position.x = req.target_pose.x if req.target_pose.move_x else xarm_pose.pose.position.x
		xarm_pose.pose.position.y = req.target_pose.y if req.target_pose.move_y else xarm_pose.pose.position.y
		xarm_pose.pose.position.z = req.target_pose.z if req.target_pose.move_z else xarm_pose.pose.position.z

		try:
			xarm_pose = self.tfBuffer.transform(xarm_pose,'BaseBrazo')
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			rospy.logerr("TF transform failed with error: %s", e)
			return MovePoseResponse(False)
		
		xarm_pose.pose.position.x *= scale_factor
		xarm_pose.pose.position.y *= scale_factor
		xarm_pose.pose.position.z *= scale_factor

		print("Moving XYZ")
		xarm.set_mode_cartesian()
		xarm.xarm_move_to_point(xarm_pose.pose.position.x, xarm_pose.pose.position.y, xarm_pose.pose.position.z)
		xarm.set_mode_moveit()
		return MovePoseResponse(True)

	def euler_to_quaternion(self, roll, pitch, yaw):
		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		return quaternion

if __name__ == "__main__":
	rospy.init_node('cartesian_server_2')
	xarm = arm()
	xarm.set_mode_moveit()
	Arm_server = ArmServices()
	#Arm_server.pick_server()
	#Arm_server.place_server()
	#Arm_server.pour_server()
	#Arm_server.move_joint_server()
	#Arm_server.move_xyz_server()
	rospy.spin()

