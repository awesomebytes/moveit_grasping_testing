#! /usr/bin/env python
"""
Created on 20/05/14

@author: Sammy Pfeiffer
"""

import rospy
import tf
import tf.transformations as TT
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point, Quaternion
from pal_vision_msgs.msg import Gesture
import numpy
import copy

GESTURE_TOPIC = '/head_mount_xtion/gestures'
PUB_1_TOPIC = '/given_pose'
PUB_2_TOPIC = '/transformed_pose'

class TransformGesture():
    def __init__(self):
        rospy.loginfo("Subscribing to '" + GESTURE_TOPIC + "'")
        self.face_sub = rospy.Subscriber(GESTURE_TOPIC, Gesture, self.gesture_cb)
        # Debugging publishers
        self.pub1 = rospy.Publisher(PUB_1_TOPIC, PoseStamped)
        self.pub2 = rospy.Publisher(PUB_2_TOPIC, PoseStamped)

        rospy.loginfo("Getting a TransformListener...")
        self.tf_listener = tf.TransformListener()
        
        
    def gesture_cb(self, data): 
        rospy.loginfo("Got gesture:\n" + str(data))
        #data = Gesture()
        # Create a PoseStamped to debug
        gesture_ps = PoseStamped(
                                   header = Header(
                                                 stamp = data.header.stamp,
                                                 frame_id = data.header.frame_id  # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                   pose = Pose(
                                             position = data.position3D, 
                                             orientation = Quaternion(w = 1.0)
                                             )
                                   )
        self.pub1.publish(gesture_ps)
        
        # Create PointStamped to transform it
        gesture_pointstamped = PointStamped(
                                            header = Header(
                                                 stamp = rospy.Time(0),  # This is to get the latest available transform
                                                 frame_id = data.header.frame_id  # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                            point = data.position3D
                                            )

        transformed_gesture_pointstamped = self.tf_listener.transformPoint("base_link", gesture_pointstamped)
        rospy.loginfo("Transformed pose:\n" + str(transformed_gesture_pointstamped))
        # Debugging PoseStamped
        pub2_ps = PoseStamped(
                               header = Header(
                                                 stamp = transformed_gesture_pointstamped.header.stamp,
                                                 frame_id = transformed_gesture_pointstamped.header.frame_id # Should be head_mount_xtion_depth_optical_frame
                                                 ),
                                pose = Pose(
                                             position = transformed_gesture_pointstamped.point, 
                                             orientation = Quaternion(w = 1.0)
                                             )
                              )
        self.pub2.publish(pub2_ps)


if __name__ == '__main__':
    rospy.init_node("test_tf_gestures_")
    tg = TransformGesture()
    rospy.spin()