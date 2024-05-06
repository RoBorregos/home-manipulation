#! /usr/bin/env python3

import rospy
import actionlib
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceGoal


# Manipulation server has priority on object name over id
MANIPULATION_TARGET = 6
MANIPULATION_TARGET_NAME = "galletas"

class ManipulationCaller(object):
    
    def __init__(self):
        rospy.loginfo("Connecting Caller to Manipulation Server")
        self.client = actionlib.SimpleActionClient('manipulationServer', manipulationPickAndPlaceAction)
        self.client.wait_for_server()
        
        result = self.manipulation_goal(target=MANIPULATION_TARGET, target_name=MANIPULATION_TARGET_NAME)
        
        print(f"CALLER GOT RESULT: {result}")

    def manipulation_goal(self, target = -2, target_name = ""):
        class ManipulationGoalScope:
            object_ = target
            object_name_ = target_name
            result = False
            
            result_received = False
        
        def manipulation_goal_feedback(feedback_msg):
            pass
        
        def get_result_callback(state, result):
            ManipulationGoalScope.result = result.result

            ManipulationGoalScope.result_received = True
            rospy.loginfo("Manipulation Goal Finished")

        rospy.loginfo(f"Sending Manipulation Goal: ID: {ManipulationGoalScope.object_}, NAME: {target_name}")
        self.client.send_goal(
                    manipulationPickAndPlaceGoal(object_id = ManipulationGoalScope.object_,
                                                object_name = ManipulationGoalScope.object_name_,),
                    feedback_cb=manipulation_goal_feedback,
                    done_cb=get_result_callback)
        
        while not ManipulationGoalScope.result_received and not rospy.is_shutdown():
            pass
        
        return ManipulationGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('ManipulationCaller', anonymous=True)
        rospy.loginfo("ManipulationCaller initialized.")
        ManipulationCaller()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)