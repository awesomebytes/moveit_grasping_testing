#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 5 10:31:00 2013

@author: Sam Pfeiffer
"""

#from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp

rospy.init_node("pick_test")
scene = PlanningSceneInterface()
robot = RobotCommander()   

rospy.sleep(1)   

# clean the scene
scene.remove_world_object("table")
scene.remove_world_object("part")

# publish a demo scene
p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()

p.pose.position.x = 0.6
p.pose.position.y = 0.0    
p.pose.position.z = 0.45
p.pose.orientation.w = 1.0
#scene.add_box("table", p, (0.5, 1.5, 0.9))
p.pose.position.x = 0.45
p.pose.position.y = -0.2
p.pose.position.z = 0.95
scene.add_box("part", p, (0.04, 0.04, 0.1))
   
rospy.sleep(1)    
       
p = PoseStamped()    
p.header.frame_id = "base_link" 
p.header.stamp = rospy.Time.now() 
# p.pose.position.x = 0.61654
# p.pose.position.y = 0.00954
# p.pose.position.z = 0.98733
p.pose.position.x = 0.25
p.pose.position.y = -0.2
p.pose.position.z = 1.00
p.pose.orientation.x = 0.028598
p.pose.orientation.y = 0.70614
p.pose.orientation.z = -0.016945  
p.pose.orientation.w = 0.70729

grasp = Grasp()
grasp.id = "test"   
grasp.grasp_pose = p

grasp.pre_grasp_approach.direction.header.frame_id = "base_link"
grasp.pre_grasp_approach.direction.header.stamp = rospy.Time.now()
grasp.pre_grasp_approach.direction.vector.x = 1.0
grasp.pre_grasp_approach.direction.vector.y = 0.0
grasp.pre_grasp_approach.direction.vector.z = 0.0
grasp.pre_grasp_approach.desired_distance = 0.05
grasp.pre_grasp_approach.min_distance = 0.01

grasp.pre_grasp_posture.header.frame_id = "base_link" # what link do i need here?
grasp.pre_grasp_posture.header.stamp = rospy.Time.now() 
grasp.pre_grasp_posture.joint_names = ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_middle_joint"]
pos = JointTrajectoryPoint() # pre-grasp with thumb down and fingers open
pos.positions.append(2.0)
pos.positions.append(0.0)
pos.positions.append(0.0)
grasp.pre_grasp_posture.points.append(pos)


grasp.grasp_posture.header.frame_id = "base_link"
grasp.grasp_posture.header.stamp = rospy.Time.now() 
grasp.grasp_posture.joint_names = ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_middle_joint"]
pos = JointTrajectoryPoint() # grasp with all closed
pos.positions.append(2.0)
pos.positions.append(2.0)
pos.positions.append(2.0)
grasp.grasp_posture.points.append(pos)

grasp.post_grasp_retreat.direction.header.frame_id = "base_link"
grasp.post_grasp_retreat.direction.header.stamp = rospy.Time.now()
grasp.post_grasp_retreat.direction.vector.x = 0.0
grasp.post_grasp_retreat.direction.vector.y = 0.0
grasp.post_grasp_retreat.desired_distance = 0.1
grasp.post_grasp_retreat.min_distance = 0.01
grasp.allowed_touch_objects = ["table"]

grasp.max_contact_force = 0



robot.right_arm_torso.pick("part", grasp)