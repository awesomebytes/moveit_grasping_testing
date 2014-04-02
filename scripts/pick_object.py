#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on April 2 15:20:00 2014

@author: Sam Pfeiffer
"""

from moveit_commander import RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, GripperTranslation, MoveItErrorCodes
import copy
from block_grasp_generator.msg import GenerateBlockGraspsAction, GenerateBlockGraspsGoal, GenerateBlockGraspsResult
from tf import transformations
from math import radians, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name
    

def createPickupGoal(group="right_arm_torso", target="part", grasp_pose=PoseStamped(), possible_grasps=[]):
    """ Create a PickupGoal with the provided data"""
    pug = PickupGoal() # haha goal is a dog
    pug.target_name = target
    pug.group_name = group
    #pug.end_effector = "right_eef"
    #pug.end_effector = 'right_eef' # should check if i can put other links here... i dont think so :S
    pug.possible_grasps.extend(possible_grasps)
    pug.allowed_planning_time = 5.0 # Compare this to the... minute we had before!!
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 10
    #pug.allowed_touch_objects.append("all")
    #pug.attached_object_touch_links.append("all") #looks bad XD
    
    return pug

def retrieveGrasps(pose, width=0.04):
    rospy.loginfo("Connecting to grasp generator AS")
    grasps_ac = actionlib.SimpleActionClient('/grasp_generator_server/generate', GenerateBlockGraspsAction)
    grasps_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    goal = GenerateBlockGraspsGoal()
    goal.pose = pose
    goal.width = width
    grasps_ac.send_goal(goal)
    rospy.loginfo("Sent goal, waiting:\n" + str(goal))
    t_start = rospy.Time.now()
    grasps_ac.wait_for_result()
    t_end = rospy.Time.now()
    t_total = t_end - t_start
    rospy.loginfo("Result received in " + str(t_total.to_sec()))
    
    grasp_list = grasps_ac.get_result().grasps
    return grasp_list


if __name__=='__main__':
    rospy.init_node("pick_object_test")
    
    rospy.loginfo("Connecting to pickup AS")
    pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)   
    
    rospy.loginfo("Cleaning world objects")
    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_link'
    p.header.stamp = rospy.Time.now()
    
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0    
    p.pose.position.z = 0.65
    p.pose.orientation.w = 1.0
    scene.add_box("table", p, (0.5, 1.5, 0.9))
    p.pose.position.x = 0.4
    p.pose.position.y = -0.3
    p.pose.position.z = 1.15
    
    angle = radians(80) # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    p.pose.orientation = Quaternion(*quat.tolist())
    
    
    scene.add_box("part", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added object to world")
    
       
    rospy.sleep(1)    
           
    pose_grasp = copy.deepcopy(p)
    possible_grasps = retrieveGrasps(pose_grasp.pose) # using grasp generator AS
    goal = createPickupGoal("right_arm_torso", "part", pose_grasp, possible_grasps)
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
    
    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()

    rospy.loginfo("Result is:")
    print result
    rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))

    
    

