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
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult


if __name__=='__main__':
    rospy.init_node("remove_world_objects")
    
    
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)   
    
    rospy.loginfo("Cleaning world objects")
    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    

