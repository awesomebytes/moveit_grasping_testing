#! /usr/bin/env python

"""
Created on April 2 15:28:00 2014

@author: Sam Pfeiffer
"""

import rospy
import actionlib
import copy
from moveit_commander import PlanningSceneInterface

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PickupAction

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from pick_object import retrieveGrasps, createPickupGoal
from place_object import createPlaceGoal
# This does not work and I think it should work :/
# from moveit_grasping_testing.pick_object import retrieveGrasps, createPickupGoal
# from moveit_grasping_testing.place_object import createPlaceGoal

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


if __name__=='__main__':
    rospy.init_node("pick_and_place")
    
    rospy.loginfo("Connecting to pickup AS")
    pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    
    rospy.loginfo("Connecting to place AS")
    place_ac = actionlib.SimpleActionClient('/place', PlaceAction)
    place_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.loginfo("Setting up planning scene interace.")
    scene = PlanningSceneInterface()
    rospy.sleep(1)   
    
    rospy.loginfo("Done, cleaning world objects from possible previous runs (table, part)")
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_link'
    p.header.stamp = rospy.Time.now()
    
    # Table position
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0    
    p.pose.position.z = 0.55
    p.pose.orientation.w = 1.0
    
    scene.add_box("table", p, (0.5, 1.5, 0.9))
    rospy.loginfo("Added 'table' to world.")
    
    # Object position
    p.pose.position.x = 0.4
    p.pose.position.y = -0.4
    p.pose.position.z = 1.10
    
    scene.add_box("part", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added 'part' to world.")
    
    rospy.sleep(1)
    
    rospy.loginfo("Creating pickup goal.")
    pose_grasp = copy.deepcopy(p)
    possible_grasps = retrieveGrasps(pose_grasp.pose)
    pickup_goal = createPickupGoal("right_arm_torso", "part", pose_grasp, possible_grasps)
    rospy.loginfo("=== Sending pickup goal. ===")
    pickup_ac.send_goal(pickup_goal)
     
    rospy.loginfo("Waiting for result...")
    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()
    rospy.loginfo("Pickup human readable error: " + str(moveit_error_dict[result.error_code.val]))
    
    if moveit_error_dict[result.error_code.val] != "SUCCESS":
        rospy.loginfo("Can't place as pickup operation failed, try again.")
        exit(0)
    
    # Place position, orientations will be generated at creating the place goal
    p.pose.position.x = 0.35
    p.pose.position.y = -0.45
    p.pose.position.z = 1.15
    
    
    rospy.loginfo("\nCreating place goal.")
    place_goal = createPlaceGoal(p, "right_arm_torso", "part")
    rospy.loginfo("=== Sending place goal. ===")
    place_ac.send_goal(place_goal)
    rospy.loginfo("Waiting for result...")
     
    place_ac.wait_for_result()
    result = place_ac.get_result()
    
    rospy.loginfo("Place human readable error: " + str(moveit_error_dict[result.error_code.val]))
    
    