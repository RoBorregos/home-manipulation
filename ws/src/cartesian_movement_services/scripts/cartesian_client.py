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

if __name__ == "__main__":
    #degree = float(sys.argv[1])
    # object_pose = sys.argv[1]
    # destination_pose = sys.argv[2]
    # tip_pick = bool(sys.argv[3])
    object_pose = [0,0,0,0,0,0]
    destination_pose = [0,0,0,0,0,0]
    is_vertical = True
    tip_pick = True
    pick_and_place_client(object_pose,destination_pose,is_vertical,tip_pick)
