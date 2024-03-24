#!/usr/bin/env python3

import os
import roslib.packages
import rospy
import numpy as np
import matplotlib.pyplot as plt
from detection_msgs.msg import DetectionDepth, DetectionDepthArray
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

def stop_head():
    os.system("rosnode kill /pal_head_manager")
    os.system("""rostopic pub -1 /head_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 20
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'head_1_joint'
- 'head_2_joint'
points:
- positions: [0, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}" """)

class DecisionNode:
    def __init__(self):
        detection_topic = rospy.get_param("~detection_topic", "detection_result")
        navigation_topic = rospy.get_param("~navigation_topic", None)
        decision_topic = rospy.get_param("~decision_topic")
        joystick_topic = rospy.get_param("~joystick_topic")
        self.classes = []
        self.depths = []
        self.joy_priority = False
        path = roslib.packages.get_pkg_dir("ultralytics_ros")
        self.decision_msg = Twist()

        self.decision_pub = rospy.Publisher(decision_topic, Twist, queue_size = 1)
        self.detection_sub = rospy.Subscriber(
            joystick_topic, Bool, self.joystick_prio, queue_size=1, buff_size=2**24
        )
        self.detection_sub = rospy.Subscriber(
            detection_topic, DetectionDepthArray, self.detection_callback, queue_size=1, buff_size=2**24
        )
        self.navigation_sub = rospy.Subscriber(navigation_topic, Twist, self.navigation_callback, queue_size=1, buff_size=2**24)

    def joystick_prio(self, msg):
        self.joy_prio = msg.data


    def detection_callback(self, msg):
        detections = msg.detections
        self.classes = []
        self.depths = []
        for d in detections:
            self.classes.append(d.results[0].id)
            self.depths.append(d.depth)

    def navigation_callback(self, msg):
        self.decision_msg = msg
        self.publish_decision(self.classes, self.depths)


    def publish_decision(self, classes, depths):
        if not self.joy_prio:
            tmp_dist = 2 #set to some upper limit
            for i, obj in enumerate(self.classes):
                if obj == 0: # person'
                    if depths[i] < tmp_dist:
                        tmp_dist = self.depths[i]
            if tmp_dist < 1:
                print(f"Person {tmp_dist} m away")
                self.decision_msg.linear.x = 0
                self.decision_msg.angular.z = 0
            elif tmp_dist < 2:
                print(f"Person {tmp_dist} m away")
                vel_factor = (tmp_dist-1) # depends on depth limits
                self.decision_msg.linear.x *= min(vel_factor,1)
                self.decision_msg.angular.z *= min(vel_factor,1)

            self.decision_pub.publish(self.decision_msg)


if __name__ == "__main__":
    rospy.init_node("decision_node")
    stop_head()
    node = DecisionNode()
    rospy.spin()
