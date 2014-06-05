#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thursday Aug 11 17:46:55 2013

@author: sampfeiffer
"""
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal

from multiprocessing.connection import Listener
from array import array

class topicListener():

    def __init__(self):
        self.subs = rospy.Subscriber('/right_arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, self.getGoal)
        self.goal_msg = None
        self.address = ('localhost', 46001)     # family is deduced to be 'AF_INET'
        self.listener = Listener(self.address, authkey='secret')
        rospy.sleep(1.0)
        self.attendReceivers()

    def getGoal(self, data):
        rospy.loginfo("Received msg at /right_arm_controller/follow_joint_trajectory/goal!")
        self.goal_msg = data

    def attendReceivers(self):
        rospy.loginfo("Attending receivers")
        while True:
            #try:
            conn = self.listener.accept()
            print 'connection accepted from', self.listener.last_accepted
            break
#             except:
#                 pass
            rospy.loginfo("Retrying...")
            rospy.sleep(1)
            #self.listener.close()

        
        while True:
            if self.goal_msg != None:
                rospy.loginfo("Sending msg!")
                conn.send(self.goal_msg)
                rospy.loginfo("Sent!")
                self.goal_msg = None
            rospy.sleep(0.1)
        conn.close()
            



if __name__ == '__main__':
    # TEST CONNECTION BEFORE ROS INIT NODE
    address = ('localhost', 46001)     # family is deduced to be 'AF_INET'
    listener = Listener(address, authkey='secret password')
    
    conn = listener.accept()
    print 'connection accepted from', listener.last_accepted
    
    conn.send([2.25, None, 'junk', float])
#     conn.close()
#     listener.close()
    # END TEST CONNECTION, THIS ALWAYS WORKS
    thisnode = rospy.init_node('listen_to_right_arm_goal', anonymous=True)
    rospy.sleep(5)
    conn.send([2.25, None, 'junk', float])
    conn.close()
    listener.close()
    #t = topicListener()
    





