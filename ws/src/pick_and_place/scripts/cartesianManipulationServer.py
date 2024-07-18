#! /usr/bin/env python3

import json
import math
import tf

import numpy
import pathlib
import actionlib
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
import numpy as np
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from object_detector_3d.msg import DetectObjects3DAction, DetectObjects3DGoal
from frida_manipulation_interfaces.msg import objectDetectionArray, objectDetection
from object_detector_3d.msg import GetPlacePositionAction, GetPlacePositionGoal
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import JointState
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist 
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceFeedback, manipulationPickAndPlaceResult
#from arm_server.msg import MoveArmAction, MoveArmResult, MoveArmFeedback, MoveArmGoal
#from arm_server.srv import Gripper, GripperResponse
from enum import Enum
from gpd_ros.srv import detect_grasps_samples
from gpd_ros.msg import GraspConfigList
import time
import tf.transformations as transformations

from cartesian_movement_services.srv import Pick as CartesianPick, PickRequest as CartesianPickRequest
from cartesian_movement_services.srv import Place as CartesianPlace, PlaceRequest as CartesianPlaceRequest
from cartesian_movement_services.srv import Pour as CartesianPour, PourRequest as CartesianPourRequest

class MoveItErrorCodes(Enum):
    SUCCESS = 1
    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7

ARM_ENABLE = True
VISION_ENABLE = True
MANIPULATION_ENABLE = True
HORIZONTAL_PICK_ENABLED = True
MOVE_TO_INITIAL_POSE = True

def handleIntInput(msg_ = "", range=(0, 10)):
    x = -1
    while x < range[0] or x > range[1]:
        print(msg_)
        while True:
            x = input()

            if x and x.isnumeric():
                break
        x = int(x)
    return x

class cartesianManipulationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)
        
        self.gpd_pose_publisher = rospy.Publisher("gpd_pose_test", PoseStamped, queue_size=5)
        self.target_label = ""
        self.ARM_GROUP = rospy.get_param("ARM_GROUP", "arm")
        self.GRIPPER_GROUP = rospy.get_param("GRIPPER_GROUP", "gripper")

        if VISION_ENABLE and MANIPULATION_ENABLE:
            # Toggle Octomap Service
            self.toggle_octomap = rospy.ServiceProxy('/toggle_octomap', SetBool)

        # Mechanisms | Initialize Robot Pose
        if ARM_ENABLE:
            rospy.loginfo("Waiting for MoveGroupCommander ARM_TORSO...")
            self.arm_group = moveit_commander.MoveGroupCommander(self.ARM_GROUP, wait_for_servers = 0)
            self.gripper_group = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP, wait_for_servers = 0)
        
        TEST_ARM_PLANNING = False
        if TEST_ARM_PLANNING:
            def moveARM(joints):
                ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])
                joint_state = JointState()
                joint_state.name = ARM_JOINTS
                joint_state.position = joints
                self.arm_group.go(joint_state, wait=True)
                self.arm_group.stop()
            
            moveARM([2*3.141, 0 ,0 , 0, 0])
            moveARM([0, 0 ,0 , 0, 0])
            return

        # Vision
        if VISION_ENABLE:
            self.vision2D_enable = rospy.Publisher("detectionsActive", Bool, queue_size=10)
            rospy.loginfo("Waiting for ComputerVision 3D AS...")
            self.vision3D_as = actionlib.SimpleActionClient("Detect3D", DetectObjects3DAction)
            self.vision3D_as.wait_for_server()
            rospy.loginfo("Loaded ComputerVision 3D AS...")
            self.place_vision_as = actionlib.SimpleActionClient("detect3d_place", GetPlacePositionAction)
            self.place_vision_as.wait_for_server()
            rospy.loginfo("Loaded Place ComputerVision 3D AS...")
            self.grasp_config_list = rospy.Publisher("grasp_config_list", GraspConfigList, queue_size=5)
            rospy.wait_for_service('/detect_grasps_server_samples/detect_grasps_samples')
            rospy.loginfo("Loaded Grasping Points Service...")
            # To publish target marker (pick/place points, etc.)
            self.target_debug_marker = rospy.Publisher("/debug/target_debug_marker", Marker, queue_size=5)

        self.ARM_TRANSFORM = "BaseBrazo"
        self.BASE_TRANSFORM = "base_footprint"
        # # Manipulation
        if MANIPULATION_ENABLE:
            rospy.loginfo("Waiting for /pickup_pose AS...")
            self.pick_as = actionlib.SimpleActionClient('/pickup_pose', PickAndPlaceAction)
            self.pick_as.wait_for_server()
            rospy.loginfo("Waiting for /place_pose AS...")
            self.place_as = actionlib.SimpleActionClient('/place_pose', PickAndPlaceAction)
            self.place_as.wait_for_server()
            self.pick_goal_publisher = rospy.Publisher("pose_pickup/goal", PoseStamped, queue_size=5)
            self.place_goal_publisher = rospy.Publisher("pose_place/goal", PoseStamped, queue_size=5)
            rospy.loginfo("Loaded Pick and Place AS...")
            rospy.loginfo("Connecting to /cartesian_movement_services/Pick")
            rospy.wait_for_service('/cartesian_movement_services/Pick')
            self.cartesian_pick_server = rospy.ServiceProxy('/cartesian_movement_services/Pick',CartesianPick)
            rospy.loginfo("Connected to /cartesian_movement_services/Pick")
            rospy.wait_for_service('/cartesian_movement_services/Place')
            self.cartesian_place_server = rospy.ServiceProxy('/cartesian_movement_services/Place',CartesianPlace)
            rospy.loginfo("Connected to /cartesian_movement_services/Place")
            rospy.wait_for_service('/cartesian_movement_services/Pour')
            self.cartesian_pour_server = rospy.ServiceProxy('/cartesian_movement_services/Pour',CartesianPour)
            # rospy.loginfo("Connected to /cartesian_movement_services/Pick")
            
            self.gpd_finger_markers = rospy.Publisher("gpd_finger_markers", MarkerArray, queue_size=5)
            self.tf_listener = tf.TransformListener()
        # a default
        self.pick_height = 0.3
        self.object_picked = False
        self.picked_vertical = False
        self.object_height = 0.3
        self.picked_object_height = 0.3
            
        self.ROBOT_MAX_RANGE_XY = rospy.get_param("ROBOT_MAX_RANGE_XY", 0.5)
        self.ROBOT_MAX_HORIZONTAL_DIMENSION = rospy.get_param("ROBOT_MAX_HORIZONTAL_DIMENSION", 0.1)
        self.ROBOT_HORIZONTAL_PICK_MIN_Z = rospy.get_param("ROBOT_HORIZONTAL_PICK_MIN_Z", 0.1)
        
        self.ARM_INIT = rospy.get_param("ARM_INIT", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_PREGRASP = rospy.get_param("ARM_PREGRASP", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_HOME = rospy.get_param("ARM_HOME", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_CARTESIAN_PREGRASP_VERTICAL = rospy.get_param("ARM_CARTESIAN_PREGRASP_VERTICAL", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_CARTESIAN_POSTGRASP_VERTICAL = rospy.get_param("ARM_CARTESIAN_POSTGRASP_VERTICAL", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_CARTESIAN_PREGRASP_HORIZONTAL = rospy.get_param("ARM_CARTESIAN_PREGRASP_HORIZONTAL", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_CARTESIAN_POSTGRASP_HORIZONTAL = rospy.get_param("ARM_CARTESIAN_POSTGRASP_HORIZONTAL", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.ARM_CARTESIAN_PREGRASP_V_NAME = rospy.get_param("ARM_CARTESIAN_PREGRASP_V_NAME", "arm_pregrasp_v")
        self.ARM_CARTESIAN_PREGRASP_H_NAME = rospy.get_param("ARM_CARTESIAN_PREGRASP_H_NAME", "arm_pregrasp_h")
        self.ARM_CARTESIAN_POSTGRASP_V_NAME = rospy.get_param("ARM_CARTESIAN_POSTGRASP_V_NAME", "arm_postgrasp_v")
        self.ARM_CARTESIAN_POSTGRASP_H_NAME = rospy.get_param("ARM_CARTESIAN_POSTGRASP_H_NAME", "arm_postgrasp_h")
        self.SHELF_HEIGHT = rospy.get_param("SHELF_HEIGHT", 0.3)
        
        rospy.loginfo("---------------------------------\nLOADED ALL ON MANIPULATION SERVER\n---------------------------------")
        
        # Initialize Manipulation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, manipulationPickAndPlaceAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
        if MOVE_TO_INITIAL_POSE:
            self.initARM()

        rospy.loginfo("Cartesian Manipulation Server Initialized ...")

    
    def moveARM(self, joints, speed, enable_octomap = True):
        if VISION_ENABLE and enable_octomap:
            rospy.loginfo("[WARNING] MOVING ARM WITH OCTOMAP DISABLED")
            self.toggle_octomap(False)
        ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])
        joint_state = JointState()
        joint_state.name = ARM_JOINTS
        joint_state.position = joints
        # set speed
        self.arm_group.set_max_velocity_scaling_factor(speed)
        # set RRTConnect and timeout
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(20)
        # planning attempts
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.go(joint_state, wait=True)
        self.arm_group.stop()
        if VISION_ENABLE and enable_octomap:
            self.toggle_octomap(True)
    
    def moveArmNamedPose(self, pose_name, speed = 0.15, enable_octomap = True):
        if VISION_ENABLE and enable_octomap:
            rospy.loginfo("[WARNING] MOVING ARM WITH OCTOMAP DISABLED")
            self.toggle_octomap(False)
        
        # set speed
        self.arm_group.set_max_velocity_scaling_factor(speed)
        # set RRTConnect and timeout
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(20)
        # planning attempts
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_named_target(pose_name)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        if VISION_ENABLE and enable_octomap:
            self.toggle_octomap(True)

    def initARM(self):
        # Move to a position to look at the objects
        # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.2, True)
        self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.2, True)
    
    def graspARM(self):
        ARM_GRASP = rospy.get_param("ARM_GRASP", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.moveARM(ARM_GRASP, 0.15)

    def execute_cb(self, goal):
        feedback = manipulationPickAndPlaceFeedback()
        
        target = goal.object_id
        target_detection = goal.detection
        
        object_name = goal.object_name.lower()
        if goal.object_name == "place":
            target = -5
        elif goal.object_name == "place_shelf":
            target = -6
        elif goal.object_name == "pour":
            target = -10
        
        if not VISION_ENABLE:
            self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
            return
        
        # Check if arm is in PREGRASP position, if not, move to it
        # Won't move if object is already picked
        if ARM_ENABLE:
            current_joints = self.arm_group.get_current_joint_values()
            if current_joints != self.ARM_CARTESIAN_PREGRASP_HORIZONTAL and not self.object_picked and target not in [-5, -6, -10]:
                # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
                self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)

        found = False
        if target == -5 or target == -6: #Place action
            if target == -5:
                if self.picked_vertical:
                    # self.moveARM(self.ARM_CARTESIAN_POSTGRASP_VERTICAL, 0.15)
                    self.moveArmNamedPose(self.ARM_CARTESIAN_POSTGRASP_V_NAME, 0.15)
                else:
                    # self.moveARM(self.ARM_CARTESIAN_POSTGRASP_HORIZONTAL, 0.15)
                    self.moveArmNamedPose(self.ARM_CARTESIAN_POSTGRASP_H_NAME, 0.15)
            
            found = None
            if target == -5:
                found = self.get_place_position()
            elif target == -6:
                found = self.get_place_position(min_height = goal.plane_min_height, max_height = goal.plane_max_height)
            if found:
                self.toggle_octomap(False)
                rospy.loginfo("Robot Placing " + self.target_label + " down")
                
                result = self.place_shelf(self.target_pose, "current", allow_contact_with_ = []) if target == -6 else self.place(self.target_pose, "current", allow_contact_with_ = [])
                if result != 1:
                    self.toggle_octomap(True)
                    rospy.loginfo("Place Failed")
                    # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
                    self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
                    self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
                    return
                rospy.loginfo("Robot Placed " + self.target_label + " down")
                self.toggle_octomap(True)
                if target != -6:
                    # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
                    self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
                self._as.set_succeeded(manipulationPickAndPlaceResult(result = True))
                return
            else:
                rospy.loginfo("Place Failed")
                self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
                return
        # pour
        elif target == -10:
            result = self.pour()
            if result != 1:
                self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
            print("Pour Result: ", result)
            self._as.set_succeeded(manipulationPickAndPlaceResult(result = True))
        else:
            # Get Objects:
            rospy.loginfo("Getting objects/position")
            self.target_label = ""
            if target_detection.point3D.point.x != 0 or target_detection.point3D.point.y != 0 or target_detection.point3D.point.z != 0:
                found = self.get_object(target_detection=target_detection)
            elif object_name != "":
                found = self.get_object(target_name=object_name)
            else:
                found = self.get_object(target=target)
        if not found:
            rospy.loginfo("Not Found")
            self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
            return
        rospy.loginfo("Object Extracted")
        
        TEST_GDP = False
        in_ = -1
        while TEST_GDP  and not rospy.is_shutdown() and in_ != 0:
            rospy.loginfo("Getting grasping points")
            grasping_points = self.get_grasping_points()
            rospy.loginfo("Grasping Points Found")
            rospy.loginfo("Waiting for user input")
            in_ = handleIntInput("(0) Quit, (1) Retry Same PC", range=(0, 1))
        
        result = 0
        
        
        if self.is_horizontal_possible(self.object_pose) and HORIZONTAL_PICK_ENABLED:
            rospy.loginfo("[INFO] Horizontal Pick Possible")
            result = self.pick_horizontal(self.object_pose, "current", allow_contact_with_ = [])
            
        else:
            rospy.loginfo("[INFO] Horizontal Pick Not Possible")
            grasping_points = self.get_grasping_points()
            if grasping_points is None:
                rospy.loginfo("Grasping Points Not Found")
                self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
                return
            
            if not MANIPULATION_ENABLE or not ARM_ENABLE:
                self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
                return

            self.grasp_config_list.publish(grasping_points)
            rospy.loginfo("Robot Picking " + self.target_label + " up")
            result = self.pick_vertical(self.object_pose, "current", allow_contact_with_ = [], grasping_points = grasping_points)
        
        if result != 1:
            self.toggle_octomap(True)
            rospy.loginfo("Pick Failed")
            self._as.set_succeeded(manipulationPickAndPlaceResult(result = False))
            return

        self.picked_object_height = self.object_height
        rospy.loginfo("Robot Picked " + self.target_label + " up")
        self._as.set_succeeded(manipulationPickAndPlaceResult(result = True))
        self.toggle_octomap(True)
    
    def scan(self, speed=0.2, joint=6, degrees=20):
        current_joints = self.arm_group.get_current_joint_values()
        current_joints[joint] += math.radians(degrees)
        self.moveARM(current_joints, speed)
        rospy.sleep(1)
        current_joints[joint] -= math.radians(degrees*2)
        self.moveARM(current_joints, speed)
        rospy.sleep(1)
        current_joints[joint] += math.radians(degrees)
        self.moveARM(current_joints, speed)
    
    def get_robot_joints(self):
        return self.arm_group.get_current_joint_values()
        
    def get_grasping_points(self):
        def add_default_grasp(grasp_configs):
            rpy_degrees = [180.0, 90.0, 0.0]
            rpy_rad = [math.radians(x) for x in rpy_degrees]
            quat = tf.transformations.quaternion_from_euler(rpy_rad[0], rpy_rad[1], rpy_rad[2])
            matrix = tf.transformations.quaternion_matrix(quat)
            R = numpy.eye(4)
            R[:3, :3] = matrix[:3, :3]
            approach = numpy.dot(R, numpy.array([1, 0, 0, 1]))[:3]
            approach = Vector3(approach[0], approach[1], approach[2])
            binormal = numpy.dot(R, numpy.array([0, 1, 0, 1]))[:3]
            binormal = Vector3(binormal[0], binormal[1], binormal[2])
            axis = numpy.dot(R, numpy.array([0, 0, 1, 1]))[:3]
            axis = Vector3(axis[0], axis[1], axis[2])
            # grasp_configs.grasps.append(GraspConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            return grasp_configs

        attempts = 0
        while attempts < 3:
            try:
                detect_grasps = rospy.ServiceProxy('/detect_grasps_server_samples/detect_grasps_samples', detect_grasps_samples)
                detect_grasps.wait_for_service()
                attempts += 1
                rospy.loginfo("Getting Grasping Points")
                resp = detect_grasps(self.object_cloud).grasp_configs
                if len(resp.grasps) > 0:
                    return add_default_grasp(resp)
                rospy.loginfo("No Grasping Points Found")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)
            time.sleep(0.5)
        
        return None
    
    def is_horizontal_possible(self, obj_pose):
        if self.get_object_max_dimension() > self.ROBOT_MAX_HORIZONTAL_DIMENSION:
            return False
        return True
    
    def get_object_max_dimension(self):
        # Object should not exceed a maximum dimension in x and y axis to allow horizontal pick
        object_cloud = self.object_point_cloud
        point_cloud_array = []
        for p in pc2.read_points(object_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            # append point xyz to array
            if not np.isnan(p[0]) and not np.isnan(p[1]):
                point_cloud_array.append([p[0], p[1], p[2]])
        point_cloud_array = np.array(point_cloud_array)
        
        # get max and min in x and y, use magnitude of the difference
        max_x = np.max(point_cloud_array[:,0])
        min_x = np.min(point_cloud_array[:,0])
        max_y = np.max(point_cloud_array[:,1])
        min_y = np.min(point_cloud_array[:,1])
        max_z = np.max(point_cloud_array[:,2])
        min_z = np.min(point_cloud_array[:,2])
        
        print(f"Max_x: {max_x}, Min_x: {min_x}, Max_y: {max_y}, Min_y: {min_y}, Max_z: {max_z}, Min_z: {min_z}")
        
        length_x = max_x - min_x
        length_y = max_y - min_y
        
        rospy.loginfo(f"[INFO] Object has max dimension in x: {max_x - min_x} and y: {max_y - min_y}")
        
        return max(max_x - min_x, max_y - min_y)

    def base_to_arm_transform(self, pose):
        rospy.loginfo("Getting Transform from " + self.ARM_TRANSFORM + " to " + self.BASE_TRANSFORM)
        self.tf_base_to_arm = self.tf_listener.lookupTransform(self.ARM_TRANSFORM, self.BASE_TRANSFORM, rospy.Time(0))
        self.base_to_arm_euler = transformations.euler_from_quaternion(self.tf_base_to_arm[1])
        rospy.loginfo(f"[INFO] Base to Arm Transform: {self.tf_base_to_arm}")
        rotation_quat = [self.tf_base_to_arm[1][0], self.tf_base_to_arm[1][1], self.tf_base_to_arm[1][2], self.tf_base_to_arm[1][3]]
        # rotate pick pose with rotation quaternion, to adjust for orientation differences bewteen base and arm
        rotation_matrix = transformations.quaternion_matrix(rotation_quat)[:3, :3]
        # rotate translations
        # self.tf_base_to_arm[0][0], self.tf_base_to_arm[0][1], self.tf_base_to_arm[0][2] = np.dot(rotation_matrix, np.array([self.tf_base_to_arm[0][0], self.tf_base_to_arm[0][1], self.tf_base_to_arm[0][2]]))
        # Transform object pose to arm base
        pose.position.x, pose.position.y, pose.position.z = np.dot(rotation_matrix, np.array([pose.position.x, pose.position.y, pose.position.z]))
        pose.position.x += self.tf_base_to_arm[0][0]
        pose.position.y += self.tf_base_to_arm[0][1]
        pose.position.z += self.tf_base_to_arm[0][2]
        return pose
    
    def pick_vertical(self, obj_pose, obj_name, allow_contact_with_ = [], grasping_points = []):

        rospy.loginfo("Pick Action")
        
        if len(grasping_points.grasps) == 0:
            rospy.loginfo("No Grasping Points Found")
            return 0
        
        on_range = False
        
        for grasp in grasping_points.grasps:
            # move arm directly 
            self.arm_group.set_max_velocity_scaling_factor(0.15)
            self.arm_group.set_planner_id("RRTConnect")
            self.arm_group.set_planning_time(20)
            self.arm_group.set_num_planning_attempts(10)
            
            pose = self.gpd_to_pose(grasp, self.object_pose)
            print("GOT POSE: ", pose)
            
            # if pose in z obtained from GPD is higher than the highest point in the object mesh, take the highest point in the object mesh
            object_cloud = self.object_point_cloud
            #print(point_cloud)
            point_cloud_array = []
            for p in pc2.read_points(object_cloud, field_names = ("x", "y", "z"), skip_nans=True):
                # append point xyz to array
                if not np.isnan(p[0]) and not np.isnan(p[1]):
                    point_cloud_array.append([p[0], p[1], p[2]])
            point_cloud_array = np.array(point_cloud_array)
            
            highest_z_value = np.max(point_cloud_array[:,2])
            print(f"Highest Z in cloud: {highest_z_value}")
            
            if pose.pose.position.z > highest_z_value:
                pose.pose.position.z = highest_z_value
            
            print("---------------------------------")
            print("PLANE HEIGHT IS ", self.plane_height)
            
            # GPD obtains the poses to draw fingers like this, so it obtaines the left and right finger positions which we need to calculate an end effector yaw
            # left_bottom = hands[i]->getPosition() - hw * hands[i]->getBinormal();
            # right_bottom = hands[i]->getPosition() + hw * hands[i]->getBinormal();
            # left_top = left_bottom + hand_depth_ * hands[i]->getApproach();
            # right_top = right_bottom + hand_depth_ * hands[i]->getApproach();
            # left_center = left_bottom + 0.5*(left_top - left_bottom);
            # right_center = right_bottom + 0.5*(right_top - right_bottom);
            # base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*hands[i]->getApproach();
            # approach_center = base_center - 0.04*hands[i]->getApproach();
            # ignore hw as its just scaling
            left_bottom = [grasp.position.x - grasp.binormal.x, grasp.position.y - grasp.binormal.y, grasp.position.z - grasp.binormal.z]
            right_bottom = [grasp.position.x + grasp.binormal.x, grasp.position.y + grasp.binormal.y, grasp.position.z + grasp.binormal.z]
            left_top = [left_bottom[0] + 0.1 * grasp.approach.x, left_bottom[1] + 0.1 * grasp.approach.y, left_bottom[2] + 0.1 * grasp.approach.z]
            right_top = [right_bottom[0] + 0.1 * grasp.approach.x, right_bottom[1] + 0.1 * grasp.approach.y, right_bottom[2] + 0.1 * grasp.approach.z]
            left_center = [left_bottom[0] + 0.5 * (left_top[0] - left_bottom[0]), left_bottom[1] + 0.5 * (left_top[1] - left_bottom[1]), left_bottom[2] + 0.5 * (left_top[2] - left_bottom[2])]
            right_center = [right_bottom[0] + 0.5 * (right_top[0] - right_bottom[0]), right_bottom[1] + 0.5 * (right_top[1] - right_bottom[1]), right_bottom[2] + 0.5 * (right_top[2] - right_bottom[2])]
            
            # graph both fingers as 3D markers
            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "gpd_fingers"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = left_center[0]
            marker.pose.position.y = left_center[1]
            marker.pose.position.z = left_center[2]
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(2)
            marker_array.markers.append(marker)
            
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "gpd_fingers"
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = right_center[0]
            marker.pose.position.y = right_center[1]
            marker.pose.position.z = right_center[2]
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(2)
            marker_array.markers.append(marker)
            
            self.gpd_finger_markers.publish(marker_array)
            
            pose_publish_msg = PoseStamped()
            pose_publish_msg.header.stamp = rospy.Time.now()
            pose_publish_msg.header.frame_id = "base_link"
            pose_publish_msg.pose = pose.pose
            self.gpd_pose_publisher.publish(pose_publish_msg)
            
            # eef_yaw is obtained from the angle of the fingers
            eef_yaw = math.atan2(left_center[1] - right_center[1], left_center[0] - right_center[0])
            rospy.loginfo("[INFO] EEF Yaw: " + str(eef_yaw))
            
            # Register the height at which the object was picked above the table, for use in place
            self.pick_height = pose.pose.position.z - self.plane_height
            rospy.loginfo("[INFO] Pick Height: " + str(self.pick_height))
            
            pick_pose = pose.pose
            rospy.loginfo(f"[INFO] Pick Pose before transformation: x={pick_pose.position.x}, y={pick_pose.position.y}, z={pick_pose.position.z} yaw={eef_yaw}")
            pick_pose = self.base_to_arm_transform(pick_pose)
            eef_yaw += self.base_to_arm_euler[2]
            
            rospy.loginfo(f"[INFO] Pick Pose after transformation: x={pick_pose.position.x}, y={pick_pose.position.y}, z={pick_pose.position.z} yaw={eef_yaw}")
            #object_pose = [self.object_pose.pose.position.x * 1000, self.object_pose.pose.position.y * 1000, self.object_pose.pose.position.z * 1000, 0, 0, eef_yaw]
            object_pose = [pick_pose.position.x * 1000, pick_pose.position.y * 1000, pick_pose.position.z * 1000, 0, 0, eef_yaw]
            is_vertical = True
            tip_pick = False
            print("Executing cartesian pick")
            print(f"Sending object position x: {object_pose[0]}, y: {object_pose[1]}, z: {object_pose[2]}")
            if (math.sqrt(object_pose[0]**2 + object_pose[1]**2) > self.ROBOT_MAX_RANGE_XY *1000):
                rospy.loginfo("[WARN] Object out of reach, skipping grasp")
                continue
            on_range = True
            resp = self.cartesian_pick_server(object_pose, is_vertical, tip_pick)
            
            print(resp.success)
            
            if resp.success:
                rospy.loginfo("Pick Success")
                self.object_picked = True
                self.picked_vertical = True
                # self.moveARM(self.ARM_CARTESIAN_POSTGRASP_VERTICAL, 0.15)
                rospy.loginfo("Moving to Postgrasp Vertical")
                self.moveArmNamedPose(self.ARM_CARTESIAN_POSTGRASP_V_NAME, 0.15)
                rospy.loginfo("Moved to Postgrasp Vertical")
                break
            
            else:
                rospy.loginfo("Pick Failed")
                # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
                self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
        
        if not self.object_picked:
            # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
            self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
        return 1
    
    def pick_horizontal(self, obj_pose, obj_name, allow_contact_with_ = []):
        
        object_cloud = self.object_point_cloud
        
        point_cloud_array = []
        for p in pc2.read_points(object_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            # append point xyz to array
            if not np.isnan(p[0]) and not np.isnan(p[1]):
                point_cloud_array.append([p[0], p[1], p[2]])
        point_cloud_array = np.array(point_cloud_array)
        
        x_min = np.min(point_cloud_array[:,0])
        x_center = np.mean(point_cloud_array[:,0])
        y_center = np.mean(point_cloud_array[:,1])
        
        # pick at the middle or at minimum ROBOT_HORIZONTAL_PICK_MIN_Z
        z = np.mean(point_cloud_array[:,2])
        plane_height = self.plane_height
        
        rospy.loginfo(f"[INFO] Object mean height is {z}, minimum placing height is {plane_height+self.ROBOT_HORIZONTAL_PICK_MIN_Z}")
        z_pick = max(z, plane_height+self.ROBOT_HORIZONTAL_PICK_MIN_Z)
        self.pick_height = z_pick - self.plane_height
        
        rospy.loginfo(f"[INFO] Found object picking position at x: {x_min}, y: {y_center}, z: {z}")
        
        # Transform to arm frame
        pick_pose = Pose()
        pick_pose.position.x = x_center
        pick_pose.position.y = y_center
        pick_pose.position.z = z_pick
        rospy.loginfo(f"[INFO] Pick Pose before transformation: x={pick_pose.position.x}, y={pick_pose.position.y}, z={pick_pose.position.z}")
        pick_pose = self.base_to_arm_transform(pick_pose)
        rospy.loginfo(f"[INFO] Pick Pose after transformation: x={pick_pose.position.x}, y={pick_pose.position.y}, z={pick_pose.position.z}")        
        is_vertical = False
        tip_pick = False
        resp = self.cartesian_pick_server([pick_pose.position.x * 1000, pick_pose.position.y * 1000, pick_pose.position.z * 1000, 0, 0, 0], is_vertical, tip_pick)
        
        if resp.success:
                rospy.loginfo("Pick Success")
                self.object_picked = True
                self.picked_vertical = False
                # self.moveARM(self.ARM_CARTESIAN_POSTGRASP_HORIZONTAL, 0.15)
                self.moveArmNamedPose(self.ARM_CARTESIAN_POSTGRASP_H_NAME, 0.15)
            
        else:
            rospy.loginfo("Pick Failed")
            # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
            self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
        
        return 1
        
        
        
    
    def place(self, obj_pose, obj_name, allow_contact_with_ = []):
        rospy.loginfo("Place Action")
        
        # print info
        print(f"Will place at plane height: {self.plane_height}")
        print(f"Will place at pick height: {self.pick_height}")
        obj_pose.pose.position.z = self.plane_height + self.pick_height
        
        if not self.picked_vertical:
            rospy.loginfo(f"[INFO] Going for Horizontal Pick, pick height is {self.pick_height}, minimum is {self.plane_height+self.ROBOT_HORIZONTAL_PICK_MIN_Z}")
            obj_pose.pose.position.z = max(obj_pose.pose.position.z, self.plane_height + self.ROBOT_HORIZONTAL_PICK_MIN_Z)
        
        place_pose = obj_pose.pose
        self.base_to_arm_transform(place_pose)
        
        is_vertical = self.picked_vertical
        tip_pick = False
        place_pose = [place_pose.position.x * 1000, place_pose.position.y * 1000, place_pose.position.z * 1000, 0, 0, 0]
        
        print("Executing cartesian place")
        print(f"Sending object position x: {place_pose[0]}, y: {place_pose[1]}, z: {place_pose[2]}")
        
        resp = self.cartesian_place_server(place_pose, is_vertical, tip_pick)
        
        print(resp.success)
        
        if resp.success:
            rospy.loginfo("Place Success")
            return 1

        rospy.loginfo("Place Failed")
    
    def place_shelf(self, obj_pose, obj_name, allow_contact_with_ = []):
        rospy.loginfo("Place Action")
        
        # print info
        print(f"Will place at plane height: {self.plane_height}")
        
        obj_pose.pose.position.z = self.plane_height
        
        place_pose = obj_pose
        # Try to place at center of shelf
        place_pose.pose.position.z += self.SHELF_HEIGHT / 2
        
        # publish marker
        marker = Marker()
        marker.header.frame_id = place_pose.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "place_pose"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = place_pose.pose.position
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.lifetime = rospy.Duration(10)
        
        try:
            self.scan(speed=0.1, joint=5, degrees=45)
            self.scan(speed=0.1, joint=4, degrees=25)
        except:
            rospy.loginfo("Scan not completed fully")
        
        self.target_debug_marker.publish(marker)
        
        # set speed
        self.arm_group.set_max_velocity_scaling_factor(0.1)
        # set RRTConnect and timeout
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(20)
        # planning attempts
        self.arm_group.set_num_planning_attempts(10)
        # ORIENTATION WON'T MATTER
        self.arm_group.set_goal_orientation_tolerance(np.deg2rad(25))
        self.arm_group.set_pose_target(place_pose.pose)
        self.toggle_octomap(True)
        # Include octomap collisions
        
        
        # plan and execute
        plan = self.arm_group.plan()
        
        if plan:
            print("Planned but won't follow")
            self.arm_group.go(wait=True)
            rospy.loginfo("Place Success")
            self.gripper_group.set_named_target("open")
            self.gripper_group.go(wait=True)
            return 1

        rospy.loginfo("Place Failed")
        self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
        return 0
    
    def pour(self):
        rospy.loginfo("Pour Action")
        picked_object_height = self.picked_object_height
        pick_height = abs(self.pick_height)
        # get bowl object
        self.get_object(target_name="bowl")
        bowl_height = self.object_height
        # get bowl x,y
        radius = self.get_object_max_dimension() / 2
        bowl_center_x = np.mean(self.object_cloud_array[:,0])
        bowl_center_y = np.mean(self.object_cloud_array[:,1])
        table_height = self.plane_height
        
        pour_pose = Pose()
        pour_pose.position.x = bowl_center_x
        pour_pose.position.y = bowl_center_y
        left_to_right = False if bowl_center_y > 0 else True
        pour_pose.position.z = np.min(self.object_cloud_array[:,2])
        print("Pour Pose before transformation: ", pour_pose)
        pour_pose = self.base_to_arm_transform(pour_pose)
        print("Pour Pose after transformation: ", pour_pose)
        
        
        # float32[] pouring_point
        # float32 bowl_height
        # float32 bowl_radius
        # float32 object_height
        # float32 grasp_height
        # bool left_to_right
        # bool tip_pick
        
        rospy.loginfo(f"[INFO]Picked Object Height: {picked_object_height}")
        rospy.loginfo(f"[INFO]Pick Height: {pick_height}")
        rospy.loginfo(f"[INFO]Bowl Height: {bowl_height}")
        rospy.loginfo(f"[INFO]Bowl Radius: {radius}")
        rospy.loginfo(f"[INFO]Bowl Center x: {pour_pose.position.x}")
        rospy.loginfo(f"[INFO]Bowl Center y: {pour_pose.position.y}")
        rospy.loginfo(f"[INFO]Table Height: {pour_pose.position.z}")
        
        pour_request = CartesianPourRequest()
        pour_request.pouring_point = [pour_pose.position.x*1000, pour_pose.position.y*1000, pour_pose.position.z*1000 + pick_height*1000 / 2]
        pour_request.bowl_height = bowl_height*1000
        pour_request.bowl_radius = radius*1000
        pour_request.object_height = picked_object_height*1000
        pour_request.grasp_height = pick_height*1000
        pour_request.left_to_right = left_to_right
        pour_request.tip_pick = False
        
        result = self.cartesian_pour_server(pour_request.pouring_point, pour_request.bowl_height, pour_request.bowl_radius, pour_request.object_height, pour_request.grasp_height, pour_request.left_to_right, pour_request.tip_pick)
        
        # self.moveARM(self.ARM_CARTESIAN_PREGRASP_HORIZONTAL, 0.15)
        self.moveArmNamedPose(self.ARM_CARTESIAN_PREGRASP_H_NAME, 0.15)
        
        return result

    def get_object(self, target = -1, target_name = "", target_detection=None):
        
        look_for_id = True
        if target_name != "":
            look_for_id = False
        
        class GetObjectsScope:
            success = False
            detection = objectDetection()
            object_pose = []
            object_cloud = []
            object_cloud_indexed = []
            x_plane = 0.0
            y_plane = 0.0
            z_plane = 0.0
            width_plane = 0.0
            height_plane = 0.0
            object_point_cloud = []
            result_received = False
        
        def get_objects_feedback(feedback_msg):
            GetObjectsScope.objects_found_so_far = feedback_msg.status
        
        def get_result_callback(state, result):
            if result is None:
                GetObjectsScope.success = False
                GetObjectsScope.result_received = True
                return
            GetObjectsScope.success = result.success
            GetObjectsScope.object_pose = result.object_pose
            GetObjectsScope.object_cloud = result.object_cloud
            GetObjectsScope.object_cloud_indexed = result.object_cloud_indexed
            GetObjectsScope.x_plane = result.x_plane
            GetObjectsScope.y_plane = result.y_plane
            GetObjectsScope.z_plane = result.z_plane
            GetObjectsScope.width_plane = result.width_plane
            GetObjectsScope.height_plane = result.height_plane
            GetObjectsScope.object_point_cloud = result.object_point_cloud
            GetObjectsScope.result_received = True

        if target == -2: # Biggest Object
            goal = DetectObjects3DGoal(plane_min_height = 0.2, plane_max_height = 3.0) # Table
        elif target_detection is None:
            # Search for Target
            attempts = 0
            success = False
            while attempts < 3:
                try:
                    detections = rospy.wait_for_message("/detections", objectDetectionArray, timeout=5.0)
                except:
                    attempts += 1
                    continue

                if len(detections.detections) == 0:
                    attempts += 1
                    continue

                if look_for_id:
                    rospy.loginfo(f"[INFO] Looking for ID: {target}")
                else:
                    rospy.loginfo(f"[INFO] Looking for Name: {target_name}")
                for detection in detections.detections:
                    
                    if (detection.label == target and look_for_id) or (detection.labelText == target_name and not look_for_id):
                        rospy.loginfo("Target Found:" + detection.labelText)
                        self.target_label = detection.labelText
                        goal = DetectObjects3DGoal(force_object = objectDetectionArray(detections = [detection]), plane_min_height = 0.2, plane_max_height = 3.0) # Table
                        GetObjectsScope.detection = detection
                        success = True
                        break
                if success:
                    break
                attempts+=1
            if not success:
                return False
        else:
            goal = DetectObjects3DGoal(force_object = objectDetectionArray(detections = [target_detection]), plane_min_height = 0.2, plane_max_height = 3.0)

        attempts = 0
        while attempts < 3:
            self.vision3D_as.send_goal(
                goal,
                feedback_cb=get_objects_feedback,
                done_cb=get_result_callback)
        
            start_time = time.time()
            while not GetObjectsScope.result_received:
                pass
        
            if GetObjectsScope.success:
                break
            attempts += 1

        if not GetObjectsScope.success:
            return False

        self.object_pose = GetObjectsScope.object_pose
        self.object_cloud = GetObjectsScope.object_cloud
        self.object_point_cloud = GetObjectsScope.object_point_cloud
        self.object_cloud_indexed = GetObjectsScope.object_cloud_indexed
        self.plane_height = GetObjectsScope.height_plane
        
        self.object_cloud_array = []
        for p in pc2.read_points(self.object_point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            # append point xyz to array
            if not np.isnan(p[0]) and not np.isnan(p[1]):
                self.object_cloud_array.append([p[0], p[1], p[2]])
        self.object_cloud_array = np.array(self.object_cloud_array)
        
        self.object_height = np.max(self.object_cloud_array[:,2]) - np.min(self.object_cloud_array[:,2])

        print(f"Found object with result: {GetObjectsScope.success}")
        return GetObjectsScope.success
    
    def get_place_position(self, min_height = 0.2, max_height = 3.0):
        class GetPositionScope:
            success = False
            target_pose = []
            x_plane = 0.0
            y_plane = 0.0
            z_plane = 0.0
            width_plane = 0.0
            height_plane = 0.0
            result_received = False
        
        def get_position_feedback(feedback_msg):
            GetPositionScope.objects_found_so_far = feedback_msg.status
        
        def get_result_callback(state, result):
            if result is None:
                GetPositionScope.success = False
                GetPositionScope.result_received = True
                return
            GetPositionScope.success = result.success
            GetPositionScope.target_pose = result.target_pose
            GetPositionScope.x_plane = result.x_plane
            GetPositionScope.y_plane = result.y_plane
            GetPositionScope.z_plane = result.z_plane
            GetPositionScope.width_plane = result.width_plane
            GetPositionScope.height_plane = result.height_plane
            GetPositionScope.result_received = True

        goal = GetPlacePositionGoal(plane_min_height = min_height, plane_max_height = max_height)

        attempts = 0
        while attempts < 3:
            self.place_vision_as.send_goal(
                goal,
                feedback_cb=get_position_feedback,
                done_cb=get_result_callback)

            start_time = time.time()
            while not GetPositionScope.result_received:
                pass

            if GetPositionScope.success:
                break
            attempts += 1

        if not GetPositionScope.success:
            return False

        self.target_pose = GetPositionScope.target_pose
        self.plane_height = GetPositionScope.height_plane
        target_pose_distance = math.sqrt(self.target_pose.pose.position.x**2 + self.target_pose.pose.position.y**2)
        return GetPositionScope.success if target_pose_distance >0.1 else False
    
    def gpd_to_pose(self, grasp_config, base_pose) -> PoseStamped:
        # Set grasp position, translation from hand-base to the parent-link of EEF
        grasp_pose = base_pose # could already have header, time, frame_id ... 
        
        grasp_pose.pose.position.x = grasp_config.position.x
        grasp_pose.pose.position.y = grasp_config.position.y
        grasp_pose.pose.position.z = grasp_config.position.z # + 0.40

        # Rotation Matrix
        rot = numpy.array([[grasp_config.approach.x, grasp_config.binormal.x, grasp_config.axis.x],
                        [grasp_config.approach.y, grasp_config.binormal.y, grasp_config.axis.y],
                        [grasp_config.approach.z, grasp_config.binormal.z, grasp_config.axis.z]])
        # get end effector yaw from GPD
        
        
        print(rot)
        R = numpy.eye(4)
        R[:3, :3] = rot
        quat = transformations.quaternion_from_matrix(R)

        # EEF yaw-offset to its parent-link (last link of arm)
        # eef_yaw_offset = 0.0
        # offquat = transformations.quaternion_about_axis(eef_yaw_offset, (0, 0, 1))
        # quat = transformations.quaternion_multiply(quat, offquat)
        # quat = transformations.unit_vector(quat)

        # Set grasp orientation
        grasp_pose.pose.orientation.x = quat[0]
        grasp_pose.pose.orientation.y = quat[1]
        grasp_pose.pose.orientation.z = quat[2]
        grasp_pose.pose.orientation.w = quat[3]

        return grasp_pose

if __name__ == '__main__':
    rospy.init_node('manipulationServer')
    server = cartesianManipulationServer(rospy.get_name())
    rospy.spin()