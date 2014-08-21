#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 5 10:31:00 2013

@author: Sam Pfeiffer
"""

#from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped, Vector3 #, Pose
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, GripperTranslation, MoveItErrorCodes
import copy
import random
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GenerateGraspsResult

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


if __name__=='__main__':
    rospy.init_node("part2_attacher")

    
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)
    
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_link'
    
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0    
    p.pose.position.z = 0.45
    p.pose.orientation.w = 1.0
    #scene.add_box("table", p, (0.5, 1.5, 0.9))
    p.pose.position.x = 0.45
    p.pose.position.y = -0.1
    p.pose.position.z = 1.0
    #scene.add_box("part", p, (0.04, 0.04, 0.1))
    scene.add_box("part2", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added object to world")
    
       
    rospy.sleep(1)    
           


