#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 9 13:41:00 2013

@author: Sam Pfeiffer
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
    rospy.init_node("pose_goal_test")
    
    rospy.loginfo("Starting up move group commander for right arm")
    right_arm_mgc = MoveGroupCommander("right_arm_torso")
    rospy.loginfo("Creating first goal")
    goal_point = Point(0.4, -0.2, 1.1)
    goal_ori = Quaternion(0.0,0.0,0.0,1.0)
    goal_pose = Pose(goal_point, goal_ori)
    right_arm_mgc.set_pose_target(goal_pose)
    right_arm_mgc.set_pose_reference_frame('base_link')

    while True:
        rospy.loginfo("go()")
        right_arm_mgc.go()
#         goal_point.x += (random() - 0.5) / 10
#         goal_point.y += (random() - 0.5) / 10
#         goal_point.z += (random() - 0.5) / 10
        goal_point.z += 0.05
        rospy.loginfo("Setting new goal:\n " + str(goal_point))
        goal_pose = Pose(goal_point, goal_ori)
        right_arm_mgc.set_pose_target(goal_pose)

