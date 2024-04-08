#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from cartesian_movement_services.srv import *

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

if __name__ == "__main__":
    client = 3
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
        object_pose = [148,-400,360,1.57,0.7853,0]
        is_vertical = True
        tip_pick = True
        pick_client(object_pose,is_vertical,tip_pick)

    elif client == 4:
    #####################################Pick clisent################
        #Real grasping point according to xarm base reference
        #[148,-446,386]
        destination_pose = [148,-446,360,1.57,0.7853,0]
        is_vertical = False
        tip_pick = True
        place_client(destination_pose,is_vertical,tip_pick)



    

