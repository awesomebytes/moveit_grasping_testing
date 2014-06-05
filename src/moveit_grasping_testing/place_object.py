#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on April 2 12:56:00 2014

@author: Sam Pfeiffer
"""

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import GripperTranslation, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from math import radians, pi, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

    

def createPlaceGoal(place_pose, group="right_arm_torso", target="part"):
    """ Create PlaceGoal with the provided data"""
    placeg = PlaceGoal()
    placeg.group_name = group
    placeg.attached_object_name = target
    
    placeg.place_locations = createPlaceLocations(place_pose)
    
    placeg.allowed_planning_time = 5.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 10
    placeg.allow_gripper_support_collision = False
    placeg.allowed_touch_objects = ['table'] # Sometimes refuses to do the movement if this is not set
    
    #placeg.planner_id
    return placeg

def createPlaceLocations(posestamped):
    """ Create a list of PlaceLocation of the object rotated every 15deg"""
    place_locs = []
    #posestamped = PoseStamped()
    # Generate all the orientations every 15 deg
    for yaw_angle in np.arange(0, 2*pi, radians(15)):
        pl = PlaceLocation()
        pl.place_pose = posestamped
        newquat = quaternion_from_euler(0.0, 0.0, yaw_angle)
        pl.place_pose.pose.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
        pl.pre_place_approach = createGripperTranslation(Vector3(0.0, 0.0, -1.0))
        pl.post_place_retreat = createGripperTranslation(Vector3(0.0, 0.0, 1.0))
    
        pl.post_place_posture = getPreGraspPosture()
        place_locs.append(pl)
        
    return place_locs
    

def createGripperTranslation(direction_vector, desired_distance=0.15, min_distance=0.01):
    """Returns a GripperTranslation message with the direction_vector and desired_distance and min_distance in it.
    Intended to be used to fill the pre_grasp_approach and post_grasp_retreat field in the Grasp message."""
    g_trans = GripperTranslation()
    g_trans.direction.header.frame_id = "base_link"
    g_trans.direction.header.stamp = rospy.Time.now()
    g_trans.direction.vector.x = direction_vector.x
    g_trans.direction.vector.y = direction_vector.y
    g_trans.direction.vector.z = direction_vector.z
    g_trans.desired_distance = desired_distance
    g_trans.min_distance = min_distance
    return g_trans


def getPreGraspPosture():
    """ Returns our pregrasp posture JointTrajectory message to fill Grasp message"""
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "base_link" # what link do i need here?
    pre_grasp_posture.header.stamp = rospy.Time.now() 
    pre_grasp_posture.joint_names = ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_middle_joint"]
    pos = JointTrajectoryPoint() # pre-grasp with thumb down and fingers open
    pos.positions.append(1.5)
    pos.positions.append(0.0)
    pos.positions.append(0.0)
    pos.time_from_start = rospy.Duration(3.0)
    pre_grasp_posture.points.append(pos)

    return pre_grasp_posture


if __name__=='__main__':
    rospy.init_node("place_object_test")
    
    rospy.loginfo("Connecting to place AS")
    place_ac = actionlib.SimpleActionClient('/place', PlaceAction)
    place_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
     
    
    rospy.sleep(1)   

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_link'
    p.header.stamp = rospy.Time.now()
    p.pose.orientation.w = 1.0
    p.pose.position.x = 0.4
    p.pose.position.y = -0.3
    p.pose.position.z = 1.15
    
    
    goal = createPlaceGoal(p, "right_arm_torso", "object_to_grasp")
    rospy.loginfo("Sending goal")
    place_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
     
    place_ac.wait_for_result()
    result = place_ac.get_result()
    
    rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))
 
     
     

