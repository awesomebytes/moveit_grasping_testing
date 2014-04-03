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
import pickle
import os

class topicReceiver():

    def __init__(self):
        rospy.loginfo("Connecting to listener process")
                
        rospy.loginfo("Connecting with right arm AS")
        self.right_arm_as = actionlib.SimpleActionClient('/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.right_arm_as.wait_for_server()
        rospy.loginfo("Connected")
        self.receiverLoop()
    
    def receiverLoop(self):
        while True:
            try:
                input_msg = pickle.load( open( "/dev/shm/right_arm_goal.p", "rb" ) )
                os.remove("/dev/shm/right_arm_goal.p")
            except IOError:
                print "file not there..."
                rospy.sleep(0.1)
                continue
            print input_msg
            goal = FollowJointTrajectoryGoal()
            rospy.loginfo("input_msg.goal.goal_time_tolerance: \n" + str(input_msg.goal.goal_time_tolerance ) )
            goal.goal_time_tolerance = rospy.Duration(secs=0, nsecs=0)
            rospy.loginfo("input_msg.goal.goal_tolerance: \n" + str(input_msg.goal.goal_tolerance ) )
            goal.goal_tolerance = input_msg.goal.goal_tolerance 
            rospy.loginfo("input_msg.goal.path_tolerance: \n" + str(input_msg.goal.path_tolerance ) )
            goal.path_tolerance = input_msg.goal.path_tolerance
            rospy.loginfo("input_msg.goal.trajectory.header: \n" + str(input_msg.goal.trajectory.header ) )
            goal.trajectory.header = input_msg.goal.trajectory.header
            goal.trajectory.header.frame_id = '/base_link'
            goal.trajectory.joint_names = input_msg.goal.trajectory.joint_names
            
            input_times = pickle.load( open( "/dev/shm/right_arm_goal_times.p", "rb" ) )
            rospy.loginfo("input_times is:\n" + str(input_times))
            
            for input_point, time in zip(input_msg.goal.trajectory.points, input_times):
                p = JointTrajectoryPoint()
                p.accelerations = input_point.accelerations
                p.positions = input_point.positions
                p.velocities = input_point.velocities
#                 rospy.loginfo("???????????????????????????????????????????????????????????????")
#                 rospy.loginfo("Time from start is:\n" + str(input_point.time_from_start))
#                 rospy.loginfo("time is:\n" + str(time))
#                 rospy.loginfo("???????????????????????????????????????????????????????????????")
                p.time_from_start = time
                goal.trajectory.points.append(p)
            
            os.remove("/dev/shm/right_arm_goal_times.p")
            #print "Goal will be:"
            rospy.loginfo("!!!!!!!!!!!Goal:\n" +str(goal))
            self.right_arm_as.send_goal(goal)
            rospy.loginfo("Waiting for result")
            self.right_arm_as.wait_for_result()
            rospy.loginfo("Goal done")

if __name__ == '__main__':
    rospy.init_node('receive_right_arm_goal_and_send', anonymous=True)
    j = JointTrajectoryPoint()
    print "JointTrajectoryPoint is:"
    print j
    t = topicReceiver()
    