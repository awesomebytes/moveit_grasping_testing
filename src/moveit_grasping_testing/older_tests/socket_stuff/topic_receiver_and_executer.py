#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thursday Aug 11 17:47:00 2013

@author: sampfeiffer
"""
import rospy
import roslib
roslib.load_manifest('control_msgs')
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from multiprocessing.connection import Client


class topicReceiver():

    def __init__(self):
        rospy.loginfo("Connecting to listener process")
        self.address = ('localhost', 46001)
        self.conn = Client(self.address, authkey='secret')
                
        rospy.loginfo("Connecting with right arm AS")
        self.right_arm_as = actionlib.SimpleActionClient('/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.right_arm_as.wait_for_server()
        rospy.loginfo("Connected")
        self.receiverLoop()
        self.conn.close()
    
    def receiverLoop(self):
        while True:
            print "self.conn.recv()"
            input_msg = self.conn.recv()
            print input_msg
            goal = FollowJointTrajectoryGoal()
            goal.goal_time_tolerance = input_msg.goal_time_tolerance
            goal.goal_tolerance = input_msg.goal_tolerance 
            goal.path_tolerance = input_msg.path_tolerance
            goal.trajectory.header = input_msg.trajectory.header
            goal.trajectory.joint_names = input_msg.joint_names
            for input_point in input_msg.trajectory.points:
                p = JointTrajectoryPoint()
                p.accelerations = input_point.accelerations
                p.positions = input_point.positions
                p.velocities = input_point.velocities
                p.time_from_start = input_point.time_from_start
                goal.trajectory.points.append(p)
            print "Goal will be:"
            print goal
            #self.right_arm_as.send_goal(goal)
            rospy.loginfo("Waiting for result")
            self.right_arm_as.wait_for_result()
            rospy.loginfo("Goal done")

if __name__ == '__main__':
    
    # TEST CONNECTION BEFORE ROS INIT NODE
    address = ('localhost', 46001)
    conn = Client(address, authkey='secret password')
    
    print conn.recv()                 # => [2.25, None, 'junk', float]

    conn.close()
    # END TEST CONNECTION, THIS ALWAYS WORKS
    rospy.init_node('receive_right_arm_goal_and_send', anonymous=True)
    rospy.sleep(5)
    conn = Client(address, authkey='secret password')
    
    print conn.recv()                 # => [2.25, None, 'junk', float]

    conn.close()
    #t = topicReceiver()
    