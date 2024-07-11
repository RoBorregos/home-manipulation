#!/usr/bin/env python3

import rospy
import actionlib

from std_msgs.msg import String, Int32
from frida_manipulation_interfaces.msg import MoveJointAction, MoveJointGoal, MoveJointResult, MoveJointFeedback, moveXYZ


ARM_SERVER = "/arm_joints_as"
NAMED_POSE = "carry"

class NamedTargetExample:

    def __init__(self):
        rospy.init_node("named_target_example")
        self.arm_server_client = actionlib.SimpleActionClient(ARM_SERVER, MoveJointAction)
        self.arm_server_client.wait_for_server()
        rospy.loginfo("Server is up")
        
        rospy.loginfo("[INFO] Moving to observe position")
        self.arm_server_client.send_goal(
            MoveJointGoal(predefined_position=NAMED_POSE),
        )
        rospy.loginfo("[INFO] Waiting for the result")
        self.arm_server_client.wait_for_result()
        
        rospy.loginfo("[INFO] Moved to observe position")


if __name__ == "__main__":
    try:
        NamedTargetExample()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
