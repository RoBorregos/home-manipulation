#! /usr/bin/env python3

import rospy
import actionlib
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceGoal
from geometry_msgs.msg import PoseStamped
from enum import Enum
import time
import signal
from object_detector_3d.msg import DetectObjects3DAction, DetectObjects3DGoal
from object_detector_2d.msg import objectDetectionArray, objectDetection

from std_msgs.msg import String
import socket


def handleIntInput(msg_ = "", range=(0, 10)):
    x = range[0] - 1
    while x < range[0] or x > range[1]:
        print(msg_)
        while True:
            x = input()

            # Check Positive and Negative Numbers
            try:
                x = int(x)
                break
            except ValueError:
                continue
        x = int(x)
    return x

class ManipulationClient(object):
    
    def __init__(self):
        rospy.loginfo("Connecting to Manipulation Server")
        self.client = actionlib.SimpleActionClient('manipulationServer', manipulationPickAndPlaceAction)
        self.client.wait_for_server()
        #self.listener = rospy.Subscriber("/manipulation_publish_goal", String, self.receivedObj)
        #self.talker = rospy.Publisher("/manipulation_publish_result", String, queue_size=20)
        self.listener = rospy.Subscriber("manipulation/goal", String, self.receivedObj)
        self.manual_pick_listener = rospy.Subscriber('/debug/selected_detection', objectDetection, self.receive_manual_pick)
        self.talker = rospy.Publisher("manipulation/response", String, queue_size=20)
        rospy.loginfo("Connected to Manipulation Server")
        
        rospy.sleep(7)

        in_ = -1
        excepted = False
        while not rospy.is_shutdown() and in_ != 500:
            excepted = False
            ## Wait for user input
            try: 
                detections = rospy.wait_for_message("/detections", objectDetectionArray, timeout=10.0)
            except rospy.exceptions.ROSException:
                print("No objects detected")
                excepted = True
            
            if excepted:
                in_ = handleIntInput("(1) Zucaritas, (2) Coca-Cola, (3) Harpic, (-2 Refresh, -1 Biggest , 500 to exit):", range=(-100, 100))
            else:
                print("Detected objects:")
                for i, detection in enumerate(detections.detections):
                    print(f"({detection.label}) {detection.labelText}")

                in_ = handleIntInput("Select object to pick (-2 Refresh, -1 Biggest , -5 Place, -10 Pour, 500 to exit): ", (-100, 100))
            if in_ == -2:
                continue
            
            self.manipulation_goal(in_)

    def receivedObj(self, msg):
        result = False
        in_ = -1
        excepted = False
        if (msg.data == "Place"):
            result = self.manipulation_goal(-5)
        
        else:
            try: 
                detections = rospy.wait_for_message("/detections", objectDetectionArray, timeout=10.0)
            except rospy.exceptions.ROSException:
                print("No objects detected")
                excepted = True
            
            if not excepted:
                print("Detected objects:")
                for i, detection in enumerate(detections.detections):
                    print(f"({detection.label}) {detection.labelText}")
                    if msg.data == detection.labelText:
                        in_ = detection.label
                print("Selected object: ", in_, " ", msg.data)
                if in_ == -1:
                    print("Object not found")
                    self.talker.publish(String("False"))
                    return
                result = self.manipulation_goal(in_)

        if result:
            self.talker.publish(String("True"))
        else:
            self.talker.publish(String("False"))
            
    def receive_manual_pick(self, msg):
        self.point_manipulation_goal(msg)
        
    def point_manipulation_goal(self, detection):
        class ManipulationGoalScope:
            object_ = detection
            result = False
            
            result_received = False
        
        def manipulation_goal_feedback(feedback_msg):
            pass
    
        def get_result_callback(state, result):
            ManipulationGoalScope.result = result.result

            ManipulationGoalScope.result_received = True
            rospy.loginfo("Manipulation Goal Finished")
        
        rospy.loginfo(f"Sending Manipulation Goal for {ManipulationGoalScope.object_.label}")
        goal = manipulationPickAndPlaceGoal(object_id = ManipulationGoalScope.object_.label, point3D = ManipulationGoalScope.object_.point3D)
        self.client.send_goal(
                    manipulationPickAndPlaceGoal(object_id = ManipulationGoalScope.object_.label, point3D = ManipulationGoalScope.object_.point3D),
                    feedback_cb=manipulation_goal_feedback,
                    done_cb=get_result_callback)


    def manipulation_goal(self, target = 1):
        class ManipulationGoalScope:
            object_ = target
            result = False
            
            result_received = False
        
        def manipulation_goal_feedback(feedback_msg):
            pass
        
        def get_result_callback(state, result):
            ManipulationGoalScope.result = result.result

            ManipulationGoalScope.result_received = True
            rospy.loginfo("Manipulation Goal Finished")

        rospy.loginfo("Sending Manipulation Goal")
        self.client.send_goal(
                    manipulationPickAndPlaceGoal(object_id = ManipulationGoalScope.object_),
                    feedback_cb=manipulation_goal_feedback,
                    done_cb=get_result_callback)
        
        while not ManipulationGoalScope.result_received and not rospy.is_shutdown():
            pass
        
        return ManipulationGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('ManipulationGoalClient', anonymous=True)
        rospy.loginfo("ManipulationGoalClient initialized.")
        ManipulationClient()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)