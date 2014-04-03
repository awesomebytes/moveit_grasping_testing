#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 9 14:43:00 2013

@author: Sam Pfeiffer


MORE GOALS GET IGNORED
"""

#from moveit_commander import MoveGroupCommander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
import actionlib
from shape_msgs.msg import SolidPrimitive
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from random import random
from std_msgs.msg import Header

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name



if __name__=='__main__':
    rospy.init_node("pose_goal_test11")
    
    rospy.loginfo("Starting up move group commander for right arm")
    right_arm_mgc = MoveGroupCommander("right_arm_torso")

    goal_point = Point(0.4, -0.2, 1.1)
    goal_ori = Quaternion(0.0,0.0,0.0,1.0)
    right_arm_mgc.set_pose_reference_frame('base_link')
    list_goals = []
    for i in range(1):
        goal_point.z += 0.05
        rospy.loginfo(str(i) + ": Setting new goal:\n " + str(goal_point))
        list_goals.append(Pose(goal_point, goal_ori))
        
    rospy.loginfo("list of goals:\n" + str(list_goals))
    right_arm_mgc.set_pose_targets(list_goals)
    rospy.loginfo("go()")
    right_arm_mgc.go()

