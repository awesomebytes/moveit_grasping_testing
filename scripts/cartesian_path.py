#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 9 14:43:00 2013

@author: Sam Pfeiffer

Was hard to get it to work and... it doesn't work, only moves first joint of the message

https://groups.google.com/forum/#!topic/moveit-users/f-Ah9O9xJhw
"""

#from moveit_commander import MoveGroupCommander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PoseArray
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes, RobotTrajectory
import actionlib
from shape_msgs.msg import SolidPrimitive
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from random import random
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name



if __name__=='__main__':
    rospy.init_node("cartesianpathtest")
    
    rospy.loginfo("Starting up move group commander for right arm")
    right_arm_mgc = MoveGroupCommander("right_arm_torso")

    #init_pose = right_arm_mgc.get_current_pose()
    init_pose = Pose(Point(0.4, -0.2, 1.2),Quaternion(0.0,0.0,0.0,1.0))
    goal_point = Point(0.45, -0.2, 1.4)
    goal_ori = Quaternion(0.0,0.0,0.0,1.0)
    goal_pose = Pose(goal_point, goal_ori)
    right_arm_mgc.set_pose_reference_frame('base_link')
    rospy.loginfo("Planning on frame:" + str(right_arm_mgc.get_planning_frame()))
    waypoints = []
    waypoints.append(init_pose)#.pose)
    waypoints.append(goal_pose)
    
    eef_step_max = 0.05
    jump_thresh_max = 10000
    while True:
        curr_eef_step = eef_step_max * random()
        curr_jump_thresh = jump_thresh_max * random()
        #path, fraction = right_arm_mgc.compute_cartesian_path(waypoints, curr_eef_step, curr_jump_thresh, False)
        path, fraction = right_arm_mgc.compute_cartesian_path(waypoints, 0.01, 1.2, False)

        rospy.loginfo("Fraction and path:")
        rospy.loginfo(str(fraction))
        if fraction <= 0:
            rospy.logerr("Couldnt compute path with eef_step:"+ str(curr_eef_step) + " and curr_jump_thresh: " + str(curr_jump_thresh))
        else:
            rospy.logwarn("!!!!! Found a solution with eef_step: " + str(curr_eef_step) + " and curr_jump_thresh: " + str(curr_jump_thresh))
            break
    rospy.loginfo(str(path))

    #waypoints_pa = PoseArray()
    #path = RobotTrajectory()

    for point in path.joint_trajectory.points: # THIS ONLY SENDS THE MESSAGE FOR THE FIRST JOINT
        joint_stt = JointState()
        joint_stt.header.frame_id = 'base_link'
        joint_stt.header.stamp = rospy.Time.now()
#         joint_stt.name.append('arm_right_2_joint')
#         joint_stt.position.append(0.5)
#         joint_stt.velocity.append(0.0)
        joint_stt.name.extend(path.joint_trajectory.joint_names)
        joint_stt.position.extend(point.positions)
        joint_stt.velocity.extend([0.0]*len(path.joint_trajectory.joint_names))
        #joint_stt.effort.append([0.0]*len(path.joint_trajectory.joint_names))
        rospy.loginfo("jointstate is: \n" + str(joint_stt))
        right_arm_mgc.set_joint_value_target(joint_stt)
        right_arm_mgc.go()
    
#     rospy.loginfo("Publishing waypoints in PoseArray /waypoints")
#     pub_waypoints = rospy.Publisher('/waypoints', PoseArray, latch=True)
#     while True:
#         pub_waypoints.publish(waypoints_pa)
#         rospy.sleep(0.1)

    
    #rospy.loginfo("go()")
    #right_arm_mgc.go()

