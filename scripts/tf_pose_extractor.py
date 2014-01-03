#!/usr/bin/env python  
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('test_get_tf')
    rospy.sleep(3)
    listener = tf.TransformListener()
    rospy.loginfo("Starting")
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/arm_right_7_link', '/hand_right_grasping_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.loginfo("Right Trans: \n"+ str(trans))
        rospy.loginfo("Rot: \n" + str(rot))
        
        try:
            (trans,rot) = listener.lookupTransform('/arm_left_7_link', '/hand_left_grasping_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rospy.loginfo("Left Trans: \n"+ str(trans))
        rospy.loginfo("Rot: \n" + str(rot))     
        
        
        rate.sleep()