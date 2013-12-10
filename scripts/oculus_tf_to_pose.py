#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 10 02:53:00 2013

@author: Sam Pfeiffer
"""


from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import rospy
import tf
import math
from tf.transformations import euler_from_quaternion
# import tf things


if __name__ == '__main__':
    rospy.init_node('tf_oculus_to_pose')
    listener = tf.TransformListener()
    oculus_pub = rospy.Publisher('/oculus_pose', PoseStamped)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/oculus', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            continue
        
        curr_pose = PoseStamped()
        curr_pose.header.frame_id = 'base_link'
        curr_pose.header.stamp = rospy.Time.now()
        #print "rot: \n" + str(rot)
        curr_pose.pose.orientation = Quaternion(*rot)
        curr_pose.pose.position = Point(0.1, 0.0, 1.6)
        oculus_pub.publish(curr_pose)

        rate.sleep()