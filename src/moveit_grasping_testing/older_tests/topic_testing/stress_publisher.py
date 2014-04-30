#! /usr/bin/env python
"""
Created on 30/04/14

@author: Sam Pfeiffer
@email: sam.pfeiffer@pal-robotics.com
"""


import rospy
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node("stress_topic_test_pub")
    rospy.sleep(1)  # 1s to catch up with time, so we can do rospy.Time.now()

    pub = rospy.Publisher('/stress_topic', Header)

    counter = 0
    while not rospy.is_shutdown():
        h = Header(seq=counter, stamp=rospy.Time.now())
        pub.publish(h)
        print h
        counter += 1
        #rospy.sleep(0.001)
