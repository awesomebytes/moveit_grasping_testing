#! /usr/bin/env python
"""
Created on 30/04/14

@author: Sam Pfeiffer
@email: sam.pfeiffer@pal-robotics.com
"""


import rospy
from std_msgs.msg import Header
from pal_interaction_msgs.msg import ASREvent
ASREvent.EVENT_LISTEN_STATE

def stress_topic_cb(data):
    rospy.sleep(0.1)
    print data

if __name__ == '__main__':
    rospy.init_node("stress_topic_test_subs")

    pub = rospy.Subscriber('/stress_topic', Header, stress_topic_cb , queue_size=1) # default queue size blocks topic

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
