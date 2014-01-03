#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thursday Aug 11 17:46:55 2013

@author: sampfeiffer
"""
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import pickle

class topicListener():

    def __init__(self):
        self.subs = rospy.Subscriber('/right_arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, self.getGoal)
        self.goal_msg = None
        self.attendReceivers()

    def getGoal(self, data):
        rospy.loginfo("Received msg at /right_arm_controller/follow_joint_trajectory/goal!")
        self.goal_msg = data
        rospy.loginfo(str(data))

    def attendReceivers(self):
        rospy.loginfo("Attending receivers")
        while True:
            if self.goal_msg != None:
                rospy.loginfo("Writing to file!")
                pickle.dump( self.goal_msg, open( "/dev/shm/right_arm_goal.p", "wb" ) )
                times = []
                for point in self.goal_msg.goal.trajectory.points:
                    times.append(point.time_from_start)
                pickle.dump( times, open( "/dev/shm/right_arm_goal_times.p", "wb" ) )
                rospy.loginfo("Wrote!")
                self.goal_msg = None
            rospy.sleep(0.1)
            



if __name__ == '__main__':
    thisnode = rospy.init_node('listen_to_right_arm_goal', anonymous=True)
    j = JointTrajectoryPoint()
    print "JointTrajectoryPoint is:"
    print j
    t = topicListener()
    





