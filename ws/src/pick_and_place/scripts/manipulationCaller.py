#! /usr/bin/env python3

"""
This script provides a ROS client for sending manipulation goals to the manipulation server.
It includes functionalities for sending goals with object IDs and object names.
"""

import rospy
import actionlib
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceGoal


# Manipulation server has priority on object name over id
MANIPULATION_TARGET = 6
MANIPULATION_TARGET_NAME = "galletas"

class ManipulationCaller(object):
    
    def __init__(self):
        """
        Initializes the ManipulationCaller, connects to the manipulation server, and sends a manipulation goal.
        """
        rospy.loginfo("Connecting Caller to Manipulation Server")
        self.client = actionlib.SimpleActionClient('manipulationServer', manipulationPickAndPlaceAction)
        self.client.wait_for_server()
        
        result = self.manipulation_goal(target=MANIPULATION_TARGET, target_name=MANIPULATION_TARGET_NAME)
        
        print(f"CALLER GOT RESULT: {result}")

    def manipulation_goal(self, target = -2, target_name = ""):
        """
        Sends a manipulation goal to the manipulation server with the specified target ID and target name.
        
        Args:
            target (int): The ID of the target object.
            target_name (str): The name of the target object.
        
        Returns:
            bool: The result of the manipulation goal.
        """
        class ManipulationGoalScope:
            object_ = target
            object_name_ = target_name
            result = False
            
            result_received = False
        
        def manipulation_goal_feedback(feedback_msg):
            """
            Callback function for receiving feedback from the manipulation server.
            """
            pass
        
        def get_result_callback(state, result):
            """
            Callback function for receiving the result from the manipulation server.
            """
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

# Examples and use cases for key technologies used

# Example of using rospy to create a ROS client for sending manipulation goals
def example_rospy_client():
    rospy.init_node('example_client')
    client = actionlib.SimpleActionClient('example_action', manipulationPickAndPlaceAction)
    client.wait_for_server()
    goal = manipulationPickAndPlaceGoal(object_id=1, object_name="example_object")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("Result:", result)

# Example of using actionlib to create a ROS client for sending manipulation goals
def example_actionlib_client():
    client = actionlib.SimpleActionClient('example_action', manipulationPickAndPlaceAction)
    client.wait_for_server()
    goal = manipulationPickAndPlaceGoal(object_id=1, object_name="example_object")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    print("Result:", result)
