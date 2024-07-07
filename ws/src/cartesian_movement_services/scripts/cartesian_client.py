#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from cartesian_movement_services.srv import *
from cartesian_movement_services.msg import *
import math as m

def move_end_effector_client(degree):
    rospy.wait_for_service('/cartesian_movement_services/TurnEndEffector')
    end_effector_rotation = rospy.ServiceProxy('/cartesian_movement_services/TurnEndEffector',TurnEndEffector)
    resp = end_effector_rotation(degree)
    return resp.success

def pick_and_place_client(object_pose,destination_pose,is_vertical,tip_pick):
    rospy.wait_for_service('/cartesian_movement_services/PickAndPlace')
    pick_and_place_ = rospy.ServiceProxy('/cartesian_movement_services/PickAndPlace',PickAndPlace)
    resp = pick_and_place_(object_pose,destination_pose,is_vertical,tip_pick)
    print(resp.success)
    return resp.success

def pick_and_pour_client(object_pose,destination_pose,container_height,bowl_radius,bowl_height,left_to_right,tip_pick):
    rospy.wait_for_service('/cartesian_movement_services/PickAndPour')
    pick_and_pour_ = rospy.ServiceProxy('/cartesian_movement_services/PickAndPour',PickAndPour)
    resp = pick_and_pour_(object_pose,destination_pose,bowl_height,bowl_radius,container_height,left_to_right,tip_pick)
    print(resp.success)
    return resp.success

def pick_client(object_pose,is_vertical,tip_pick):
    rospy.wait_for_service('/cartesian_movement_services/Pick')
    pick_ = rospy.ServiceProxy('/cartesian_movement_services/Pick',Pick)
    resp = pick_(object_pose,is_vertical,tip_pick)
    print(resp.success)
    return resp.success

def place_client(destination_pose,is_vertical,tip_pick):
    rospy.wait_for_service('/cartesian_movement_services/Place')
    place_ = rospy.ServiceProxy('/cartesian_movement_services/Place',Place)
    resp = place_(destination_pose,is_vertical,tip_pick)
    print(resp.success)
    return resp.success

def pour_client(destination_pose,container_height,bowl_radius,bowl_height,grasp_height,left_to_right,tip_pick):
    rospy.wait_for_service('/cartesian_movement_services/Pour')
    pour_ = rospy.ServiceProxy('/cartesian_movement_services/Pour',Pour)
    resp = pour_(destination_pose,bowl_height,bowl_radius,container_height,grasp_height,left_to_right,tip_pick)
    print(resp.success)
    return resp.success

def place_in_shelf(destination_pose,is_vertical,tip_pick):
    rospy.wait_for_service('/cartesian_movement_services/PlaceInShelf')
    place_in_shelf_ = rospy.ServiceProxy('/cartesian_movement_services/PlaceInShelf',PlaceInShelf)
    resp = place_in_shelf_(destination_pose,is_vertical,tip_pick)
    print(resp.success)
    return resp.success

def move_pose_client(x,y,z,move_x,move_y,move_z):
    rospy.wait_for_service('/cartesian_movement_services/MovePose')
    move_pose = rospy.ServiceProxy('/cartesian_movement_services/MovePose',MovePose)
    move_pose_ = moveXYZ()
    move_pose_.x = x
    move_pose_.y = y
    move_pose_.z = z
    move_pose_.move_x = move_x
    move_pose_.move_y = move_y
    move_pose_.move_z = move_z
    print('About to call service')
    print(move_pose_)
    resp = move_pose(move_pose_)
    print(resp.success)
    return resp.success

if __name__ == "__main__":
    client = 7
    if client == 0:
    #####################################Change EE orientation client################
        degree = float(sys.argv[1])
        move_end_effector_client(degree)
        object_pose = sys.argv[1]
        destination_pose = sys.argv[2]
        tip_pick = bool(sys.argv[3])
    ################################################################################
    elif client == 1:
    #####################################Pick and Place clisent################
        #Real grasping point according to xarm base reference
        #[148,-446,386]

        object_pose = [148,-446,360,1.57,0.7853,0]
        destination_pose = [-148,-446,360,1.57,0.7853,0]
        is_vertical = True
        tip_pick = True
        pick_and_place_client(object_pose,destination_pose,is_vertical,tip_pick)
    ###########################################################################
    elif client == 2:

    ##########################Pouring Cereal####################################
        tip_pick = True
        object_pose = [-230,-300,420,1.57,0.7853,0]
        destination_pose = [0,-330,420,1.57,0.7853,0]    
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
        #yaw_angle_cereal = m.radians(135-cereal_orientation)
        final_orientation = 0
            #Distance between the en effector and grasping point of the gripper (in mms)
        tip_point_offset = 175

            #Distance between the end effector and grasping-by-hug point of the gripper (in mms)
        hug_point_offset = 130
        #Position 1 (Vertical):
        #220,-450,420
        #Position 1 (Horizontal):
        #220,-380,380
        print('About to execute pick and pour')
        pick_and_pour_client(object_pose,destination_pose,container_height,bowl_radius,bowl_height,True,tip_pick)
############################################################################################################
    
    elif client == 3:
    #####################################Pick clisent################
        #Real grasping point according to xarm base reference
        #[148,-446,386]
        object_pose = [0,-480,300,1.57,0.7853,m.radians(45)]
        is_vertical = False
        tip_pick = False
        pick_client(object_pose,is_vertical,tip_pick)

    elif client == 4:
    #####################################Pick clisent################
        #Real grasping point according to xarm base reference
        #[148,-446,386]
        object_pose = [100,-480,400,1.57,0.7853,0]
        is_vertical = True
        tip_pick = False
        pick_client(object_pose,is_vertical,tip_pick)

        destination_pose = [100,-480,400,1.57,0.7853,m.radians(-35)]
        is_vertical = True
        tip_pick = False
        place_client(destination_pose,is_vertical,tip_pick)

        object_pose = [100,-380,400,1.57,0.7853,m.radians(65)]
        is_vertical = False
        tip_pick = False
        pick_client(destination_pose,is_vertical,tip_pick)

        destination_pose = [100,-380,400,1.57,0.7853,m.radians(65)]
        is_vertical = False
        tip_pick = False
        place_client(destination_pose,is_vertical,tip_pick)

        object_pose = [-200,-380,400,1.57,0.7853,m.radians(65)]
        is_vertical = False
        tip_pick = False
        pick_client(object_pose,is_vertical,tip_pick)

        destination_pose = [0,-400,300,1.57,0.7853,0]
        container_height = 120
        grasping_height = 100
        bowl_radius = 70
        bowl_height = 85
        left_to_right = True
        tip_pick = False
        pour_client(destination_pose,container_height,bowl_radius,bowl_height,grasping_height,left_to_right,tip_pick)
        
        destination_pose = [150,-350,400,1.57,0.7853,m.radians(65)]
        is_vertical = False
        tip_pick = False
        place_client(destination_pose,is_vertical,tip_pick)
    
    elif client == 5:
    #####################################Pour clisent################
        #Real grasping point according to xarm base reference
        #[148,-446,386]
        destination_pose = [0,-400,300,1.57,0.7853,0]
        container_height = 120
        grasping_height = 100
        bowl_radius = 70
        bowl_height = 85
        left_to_right = True
        tip_pick = False
        pour_client(destination_pose,container_height,bowl_radius,bowl_height,grasping_height,left_to_right,tip_pick)
    elif client == 6:
        
        print('About to execute place in shelf')
        destination_pose = [-250,-400,700,1.57,0.7853,0]
        tip_pick = True
        is_vertical = False
        place_in_shelf(destination_pose,is_vertical,tip_pick)
    elif client == 7:
        print('About to execute move pose')
        x = 0
        y = 0
        z = 1.0
        move_x = False
        move_y = False
        move_z = True
        #target_pose_test = moveXYZ(x,y,z,move_x,move_y,move_z)
        move_pose_client(x,y,z,move_x,move_y,move_z)


    

