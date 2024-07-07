#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to control the angles of the arm joints
"""

import math
import time
import rospy
import actionlib
import numpy as np
import moveit_commander
from xarm_msgs.srv import *
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import JointState
from frida_manipulation_interfaces.msg import MoveJointAction, MoveJointFeedback, MoveJointResult, MoveJointGoal
from frida_manipulation_interfaces.srv import Gripper, GripperResponse, MoveJointSDK, MoveVeloJointSDK, AimToPointSDK
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Bool

VELOCITY = 0.35
ACCELERATION = 0.025
POSITION_TOLERANCE = 0.012
ORIENTATION_TOLERANCE = 5

class ArmServer:
    """Class to control the arm joints and gripper of the robot"""
    def __init__(self):
        self.ARM_GROUP = rospy.get_param("ARM_GROUP", "arm")
        self.ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.GRIPPER_GROUP = rospy.get_param("GRIPPER_GROUP", "gripper")
        self.HEAD_GROUP = rospy.get_param("HEAD_GROUP", "head")

        # ARM GROUP STATES
        self.ARM_HOME = rospy.get_param("ARM_HOME", [0.0, 0.0, 0.0, -1.5708, 1.5708, 0.7854] )
        self.ARM_CALIBRATION = [-1.57, 0.0, -3.1416 / 4, 0, -3.1416 / 4, -2.356]
        self.ARM_NAV = [-1.5708, -1.0472, -1.0472, 1.5708, 0.0, -0.7854]
        self.ARM_PREGRASP = rospy.get_param("ARM_PREGRASP", [-1.57, 0, -3.14, 0, 1.8326, 0.7854])
        self.ARM_HRI = [-1.5708, -0.5759, -1.5708, 0.0, -0.3490, 0.6981]
        self.ARM_BACK = [1.5708, -0.5759, -1.5708, 0.0, -0.3490, 0.6981]
        self.ARM_SEAT = [-1.5708, -0.5759, -1.5708, 0.0, 0.191986, 0.6981]
        self.ARM_LEFT_SEAT = [0.0, -0.5759, -1.5708, 0.0, 0.191986, 0.6981]
        self.ARM_RIGHT_SEAT = [-3.1415, -0.5759, -1.5708, 0.0, 0.191986, 0.6981]

        self.defined_states = {
            "home": self.ARM_HOME,
            "calibration": self.ARM_CALIBRATION,
            "nav": self.ARM_NAV,
            "pregrasp": self.ARM_PREGRASP,
            "face_detection": self.ARM_HRI,
            "back": self.ARM_BACK,
            "seat": self.ARM_SEAT,
            "left_face": self.ARM_LEFT_SEAT,
            "right_face": self.ARM_RIGHT_SEAT
        }

        rospy.init_node('arm_server')
        
        self.arm_group = moveit_commander.MoveGroupCommander(self.ARM_GROUP, wait_for_servers = 0)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP, wait_for_servers = 0)
        rospy.Subscriber("/stop_arm", Bool, self.stop_arm_cb)
        self.stop_arm = False

        rospy.wait_for_service('/xarm/move_joint')
        self.joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
        self.get_angle = rospy.ServiceProxy('/xarm/get_servo_angle', GetFloat32List)
        
        #self.move_joints(self.ARM_HOME)
        self.arm_as = actionlib.SimpleActionServer(
            "/arm_joints_as", MoveJointAction, execute_cb=self.arm_cb, auto_start=False,
        )
        self.arm_as.start()

        #rospy.Subscriber("/hri_move", MoveHRI, self.hri_move)
        self.arm_as_feedback = MoveJointFeedback()
        self.arm_as_result = MoveJointResult()

        self.mode = "Moveit"
        self.state = 0
        self.gripper_server = rospy.Service("/gripper_service", Gripper, self.handle_gripper)
        self.arm_joint_sdk = rospy.Service('/move_joint_sdk', MoveJointSDK ,self.handle_move_joint_sdk)
        self.arm_joint_velocity_sdk = rospy.Service('/move_joint_velocity_sdk',MoveVeloJointSDK,self.handle_move_joint_by_velocity)
        self.aim_to_point_sdk = rospy.Service("/aim_to_point_sdk",AimToPointSDK,self.handle_aim_to_point_sdk)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.wait_for_service('/xarm/move_line')
        rospy.set_param('/xarm/wait_for_finish', True)

        rospy.spin()

    def hri_move(self, goal):
        print("Received goal: " + str(goal))
        current = self.arm_group.get_current_joint_values()
        current[0] += goal.x*3.1416/360
        current[4] += goal.y*3.1416/360

        joint_state = JointState()
        joint_state.name = self.ARM_JOINTS
        joint_state.position = current
        # set speed

        self.arm_group.stop()
        self.arm_group.go(joint_state, wait=False)

    def stop_arm_cb(self, msg):
        if msg.data == True:
            self.stop_arm = True
            self.arm_group.stop()
        else:
            self.stop_arm = False

    def arm_cb(self, goal):
        """Callback for the arm joints action server"""
        rospy.loginfo(f"Received goal: {goal}")
        init_t = time.time()
        result = MoveJointResult()
        if self.stop_arm:
            result.success = False
            self.arm_as.set_aborted( result )

        if goal.speed == 0:
            goal.speed = VELOCITY
        if goal.acceleration == 0:
            goal.acceleration = ACCELERATION
        if goal.position_tolerance == 0:
            goal.position_tolerance = POSITION_TOLERANCE
        if goal.orientation_tolerance == 0:
            goal.orientation_tolerance = ORIENTATION_TOLERANCE

        self.arm_group.set_max_velocity_scaling_factor(goal.speed)
        self.arm_group.set_max_acceleration_scaling_factor(goal.acceleration)
        self.arm_group.set_goal_position_tolerance(goal.position_tolerance)
        self.arm_group.set_goal_orientation_tolerance( np.deg2rad(goal.orientation_tolerance) )

        if goal.pose_target != Pose():
            result.success = self.plan_arm(goal.pose_target, goal.planning_time, goal.planning_attempts)
        else:
            if goal.target_delta_x != 0 or goal.target_delta_y != 0:
                # Get the current joints and only change two of them
                current = self.arm_group.get_current_joint_values()
                current[0] -= goal.target_delta_x * 3.1416 / 180
                current[4] -= goal.target_delta_y * 3.1416 / 180
                goal.joints_target = current

            elif goal.predefined_position is not None:
                if goal.predefined_position in self.defined_states:
                    goal.joints_target = self.defined_states[goal.predefined_position]

            result.success = self.move_joints(goal.joints_target)

        result.execution_time = time.time() - init_t
        if not result.success:
            self.arm_as.set_aborted( result )

        self.arm_as.set_succeeded( result )

    def move_joints(self, joint_values):
        feedback = MoveJointFeedback()
        joint_state = JointState()
        joint_state.name = self.ARM_JOINTS
        joint_state.position = joint_values
        # set speed
        
        feedback.execution_state = "Moving"
        feedback.completion_percentage = 0.0

        self.arm_as.publish_feedback( feedback )
        t = time.time()
        self.arm_group.stop()
        self.arm_group.go(joint_state, wait=False)
        current_joints = self.arm_group.get_current_joint_values()
        distance = math.sqrt( (joint_values[0] - current_joints[0])**2 + (joint_values[1] - current_joints[1])**2 + (joint_values[2] - current_joints[2])**2 + (joint_values[3] - current_joints[3])**2 + (joint_values[4] - current_joints[4])**2 + (joint_values[5] - current_joints[5])**2 )
        max_distance = distance
        print("Max distance: " + str(max_distance))

        while( feedback.completion_percentage < 0.99 and distance > 0.05):
            if( self.stop_arm or self.arm_as.is_preempt_requested() ):
                rospy.loginfo("Stopping arm")
                self.arm_group.stop()
                self.arm_as.set_preempted()
                feedback.execution_state = "Stopped"
                self.arm_as.publish_feedback( feedback )
                return False
            current_joints = self.arm_group.get_current_joint_values()
            distance = math.sqrt( (joint_values[0] - current_joints[0])**2 + (joint_values[1] - current_joints[1])**2 + (joint_values[2] - current_joints[2])**2 + (joint_values[3] - current_joints[3])**2 + (joint_values[4] - current_joints[4])**2 + (joint_values[5] - current_joints[5])**2 )
            try:
                feedback.completion_percentage = 1 - distance / max_distance
            except:
                feedback.completion_percentage = 1
            self.arm_as.publish_feedback( feedback )
            rospy.sleep(0.1)


        rospy.loginfo("Execution time: " + str(time.time() - t))
        feedback.execution_state = "Finished"
        self.arm_as.publish_feedback( feedback )
        return True        

    def plan_arm(self, pose, p_time, p_attempts):
        feedback = MoveJointFeedback()

        planners = ["ompl"]
        for planner in planners:
            self.arm_group.set_planning_pipeline_id(planner)
            self.arm_group.set_planner_id("RRTConnect")
            self.arm_group.set_planning_time(p_time)
            self.arm_group.set_num_planning_attempts(p_attempts)
            self.forget_joint_values()
            self.set_pose_target(pose)
            
            print("Planning to pose: " + str(pose))
            feedback.execution_state = "Planning"
            self.arm_as.publish_feedback( feedback )
                
            t = time.time()
            res = self.arm_group.plan()
            rospy.loginfo("Planning time: " + str(time.time() - t))
            if res[0] == False:
                rospy.loginfo("Planning failed")
                return False

        feedback.execution_state = "Moving"
        self.arm_as.publish_feedback( feedback )
        t = time.time()
        self.arm_group.go(wait=False)
        current_pose = self.arm_group.get_current_pose().pose
        pose_distance = math.sqrt( (pose.position.x - current_pose.position.x)**2 + (pose.position.y - current_pose.position.y)**2 + (pose.position.z - current_pose.position.z)**2 )
        max_distance = pose_distance

        while( not self.stop_arm and not self.arm_as.is_preempt_requested() and pose_distance > 0.1 ):
            if( self.stop_arm or self.arm_as.is_preempt_requested() ):
                rospy.loginfo("Stopping arm")
                self.arm_group.stop()
                self.arm_as.set_preempted()
                return False
            current_pose = self.arm_group.get_current_pose().pose
            pose_distance = math.sqrt( (pose.position.x - current_pose.position.x)**2 + (pose.position.y - current_pose.position.y)**2 + (pose.position.z - current_pose.position.z)**2 )
            try:
                feedback.completion_percentage = 1 - pose_distance / max_distance
            except:
                feedback.completion_percentage = 1
            self.arm_as.publish_feedback( feedback )
            rospy.sleep(0.1)

        rospy.loginfo("Execution time: " + str(time.time() - t))
        self.arm_group.stop()
        return True        
        

    def handle_gripper(self, req):
        """Open or close the gripper on user request"""
        gripper_state = req.state
        rospy.loginfo(f'Executing gripper state: {gripper_state}')
        self.gripper_group.set_named_target( gripper_state )
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        rospy.sleep(0.25)
        return GripperResponse(success=True)
    
    def return_to_default_pose_horizontal(self, request):
        rospy.wait_for_service('/xarm/move_joint')
        joint_move = rospy.ServiceProxy('/xarm/move_joint', Move)
        req = MoveRequest() 
        req.mvvelo = 0.1
        req.mvacc = 7
        req.mvtime = 0
        req.mvradii = 0
        try:
            self.set_mode_cartesian()
            self.is_vertical = False
            req.pose = [-1.5707963705062866, -0.6108652353286743, -1.5707963705062866, 3.1415927410125732, -0.6108652353286743, -2.356194496154785]
            joint_move(req)
            self.set_mode_moveit()
        except rospy.ServiceException as e:
            print("Default horizontal movement failed: %s"%e)
            self.error_status = 1

    def handle_move_joint_by_velocity(self,request):
        """Service to move the arm joints with angular velocities"""
        rospy.wait_for_service('/xarm/velo_move_joint')
        joint_velocity = rospy.ServiceProxy('/xarm/velo_move_joint', MoveVelo)
        #rospy.loginfo("Proxyd services")
        req = MoveVeloRequest()
        req.jnt_sync = 1
        req.coord = 0
        joint = request.joint_number
        speed = request.speed
        req.velocities = [0,0,0,0,0,0]
        try:
            if(self.mode != "Velocity"):
                self.set_mode_velocity()            
            req.velocities[joint] = speed
            rospy.loginfo("Final request: ")
            rospy.loginfo(req)
            joint_velocity(req)
            
        except rospy.ServiceException as e:
            self.error_status = 1
            self.mode = "Moveit"
            
        #req.velocities = [request.]

    def handle_move_joint_sdk(self, request):
        """Service to move the arm joints using the SDK"""
        req = MoveRequest() 
        req.mvvelo = request.vel
        req.mvacc = request.acc
        req.mvtime = 0
        req.mvradii = 0
        angle_getter = self.get_angle
        actual_pose = list(angle_getter().datas)
        joint_number = int(request.joint_number)
        degrees = float(request.degree)
        radians = math.radians(degrees)
        print("checking joint number")
        if joint_number == 5:
            yaw_angle = radians - math.radians(45)
            if actual_pose[5] < 0:
                yaw_angle += math.pi
            actual_pose[joint_number] = yaw_angle
        else:
            actual_pose[joint_number] = radians

        try:
            if(self.mode != "Cartesian"):
                print("SETTING CARTESIAN")
                self.set_mode_cartesian()
            req.pose = [actual_pose[0],actual_pose[1],actual_pose[2],actual_pose[3],actual_pose[4],actual_pose[5]]
            print(req)
            self.joint_move(req)
            #self.set_mode_moveit()
        except rospy.ServiceException as e:
            print("Joint movement failed: %s" % e)
            self.error_status = 1
            self.mode = "Moveit"



    def handle_aim_to_point_sdk(self,request):
        """Service to move the arm to a given point using the SDK"""
        req = MoveRequest() 
        req.mvvelo = request.vel
        req.mvacc = request.acc
        req.mvtime = 0
        req.mvradii = 0
        angle_getter = self.get_angle
        actual_pose = list(angle_getter().datas)
        joint_number = int(request.joint_number)
        point_header_id = request.target_point.header.frame_id

        self.tf_listener.waitForTransform("BaseBrazo", point_header_id, rospy.Time(), rospy.Duration(1.0))
        
        try:
            point_pose_to_xarm = self.tfBuffer.transform(request.target_point, "BaseBrazo")
            angle_towards_point = math.atan2(point_pose_to_xarm.pose.position.y, point_pose_to_xarm.pose.position.x)
            actual_pose[joint_number] = angle_towards_point
            if(self.mode != "Cartesian"):
                self.set_mode_cartesian()
            req.pose = [actual_pose[0],actual_pose[1],actual_pose[2],actual_pose[3],actual_pose[4],actual_pose[5]]
            print(req.)
            self.joint_move(req)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF lookup failed with error: %s", e)
            return AimToPointSDKResponse(False)


    def set_mode_cartesian(self):
        """Set the mode to Cartesian"""
        if self.mode != "Cartesian":
            set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
            set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
            set_mode(0)
            set_state(0)
            self.error_status = 0
            self.mode = "Cartesian"
            self.state = 0
            print("Cartesian mode set")
            time.sleep(2.0)

    def set_mode_velocity(self):
        """Set the mode to Cartesian"""
        if self.mode != "Velocity":
            set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
            set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
            set_mode(4)
            set_state(0)
            self.error_status = 0
            self.mode = "Velocity"
            self.state = 0
            print("Velocity mode set")
            time.sleep(2.0)

    def set_mode_moveit(self):
        """Set the mode to Moveit"""
        if self.mode != "Moveit":
            set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
            set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
            set_mode(1)
            set_state(0)
            self.error_status = 0
            self.mode = "Moveit"
            self.state = 0
            print("Moveit mode set")
            time.sleep(2.0)

    

if __name__ == '__main__':
    ArmServer()
