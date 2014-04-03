#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 10 02:53:00 2013

@author: Sam Pfeiffer
"""


from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import rospy
import tf
#import tf2_py
import math
from tf.transformations import euler_from_quaternion
# import tf things


if __name__ == '__main__':
    rospy.init_node('tf_oculus_to_pose')
    listener = tf.TransformListener()
    oculus_pub = rospy.Publisher('/oculus_pose', PoseStamped)
    rospy.sleep(3.0) # This sleep is needed because of a bug of tf
    rate = rospy.Rate(10.0)
    listener.waitForTransform("/oculus", "/oculus_torso_2_ref_link", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        #now = rospy.Time().now()
        listener.waitForTransform("/oculus", "/oculus_torso_2_ref_link", rospy.Time(), rospy.Duration(4.0))
        try:
            (trans,rot) = listener.lookupTransform('/oculus', '/oculus_torso_2_ref_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException):
            continue
        
        curr_pose = PoseStamped()
        curr_pose.header.frame_id = 'base_link'
        curr_pose.header.stamp = rospy.Time.now()
        #print "rot: \n" + str(rot)
        curr_pose.pose.orientation = Quaternion(*rot)
        curr_pose.pose.position = Point(0.1, 0.0, 2.0)
        oculus_pub.publish(curr_pose)

        rate.sleep()