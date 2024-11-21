#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander
from sensor_msgs.msg import JointState
import tf

class GetState():
    """
    A ROS node that retrieves and prints the current joint values of the robot arm.
    """
    ARM_GROUP = "arm"
    def __init__(self):
        """
        Initializes the GetState node, MoveGroupCommander, and TF listeners.
        """
        rospy.init_node('get_state', anonymous=True)
        self.moveit_commander = moveit_commander.MoveGroupCommander(GetState.ARM_GROUP, wait_for_servers = 0)
        self.moveit_commander.set_goal_orientation_tolerance(0.11)
        self.moveit_commander.set_goal_position_tolerance(0.01)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
    
    def run(self):
        """
        Continuously retrieves and prints the current joint values of the robot arm until the user decides to stop.
        """
        while rospy.is_shutdown() == False and input("cont: (y/n) ") != 'n':
            print('state', self.moveit_commander.get_current_joint_values())

if __name__ == '__main__':
    try:
        node = GetState()
        node.run()
    except rospy.ROSInterruptException:
        pass

# Examples and use cases for key technologies used in this node:

# 1. rospy: Used for ROS communication.
# Example: rospy.init_node('get_state', anonymous=True) initializes the ROS node.

# 2. moveit_commander: Used for planning and executing arm movements.
# Example: self.moveit_commander.get_current_joint_values() retrieves the current joint values of the robot arm.

# 3. tf: Used for transforming coordinates between different reference frames.
# Example: self.listener = tf.TransformListener() initializes the TF listener.
