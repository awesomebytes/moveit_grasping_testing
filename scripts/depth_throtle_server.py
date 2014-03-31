#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 28 09:50:00 2014

@author: Sam Pfeiffer

Throtle depth


"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse

class depth_throtle():
    def __init__(self):
        # subscribe to depth
        self.curr_depth_img = None
        self.depth_img_count = 0
        self.depth_subs = None
        # set publisher of depth
        self.throtled_pub = rospy.Publisher("/head_mount_xtion/depth/image_raw_throtled", Image)
        # create service
        self.depth_service = rospy.Service("/depth_throtle", Empty, self.callback_service)
        rospy.loginfo("Depth throtle service running.")

    def callback_depth(self, data):
        """Callback for the topic subscriber.
       Prints the current received data on the topic."""
        self.curr_depth_img = data
        self.depth_img_count += 1

    def callback_service(self, data):
        self.depth_subs = rospy.Subscriber("/head_mount_xtion/depth/image_raw", Image, self.callback_depth)
        while self.depth_img_count < 2: # wait for 2 depth images as real xtion fails with first one
            rospy.sleep(0.1)
        
        if self.curr_depth_img != None:
            self.throtled_pub.publish(self.curr_depth_img)
            rospy.loginfo("Published depth image to /head_mount_xtion/depth/image_raw_throtled")
            return EmptyResponse()
        else: # fail if there is no depth_img
            return None


if __name__=='__main__':
    rospy.init_node("throtle_depth_node")
    node = depth_throtle()

    rospy.spin()

    
    
