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

if __name__ == "__main__":
    if len(sys.argv) == 2:
        degree = float(sys.argv[1])
    else:
        sys.exit(1)
    move_end_effector_client(degree)