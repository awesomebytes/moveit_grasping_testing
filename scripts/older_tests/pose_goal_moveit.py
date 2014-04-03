#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 9 11:33:00 2013

@author: Sam Pfeiffer

rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 /base_link /odom_combined 100
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


def create_move_group_pose_goal(goal_pose=Pose(), group="right_arm_torso", end_link_name="arm_right_tool_link", plan_only=True):
    """ Creates a move_group goal based on pose.
    @arg group string representing the move_group group to use
    @arg end_link_name string representing the ending link to use
    @arg goal_pose Pose() representing the goal pose"""
    
    # Specifying the header makes the planning fail...
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = rospy.Time.now()
    moveit_goal = MoveGroupGoal()
    goal_c = Constraints()
    position_c = PositionConstraint()
    position_c.header = header
    position_c.link_name = end_link_name
    position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
    position_c.constraint_region.primitive_poses.append(goal_pose)
    position_c.weight = 1.0
    goal_c.position_constraints.append(position_c)
    orientation_c = OrientationConstraint()
    orientation_c.header = header
    orientation_c.link_name = end_link_name
    orientation_c.orientation = goal_pose.orientation
    orientation_c.absolute_x_axis_tolerance = 0.01
    orientation_c.absolute_y_axis_tolerance = 0.01
    orientation_c.absolute_z_axis_tolerance = 0.01
    orientation_c.weight = 1.0
    goal_c.orientation_constraints.append(orientation_c)
    moveit_goal.request.goal_constraints.append(goal_c)
    moveit_goal.request.num_planning_attempts = 1
    moveit_goal.request.allowed_planning_time = 5.0
    moveit_goal.planning_options.plan_only = plan_only
    moveit_goal.planning_options.planning_scene_diff.is_diff = True
    moveit_goal.request.group_name = group
    
    return moveit_goal

def append_pose_to_move_group_goal(goal_to_append=None, goal_pose=Pose(), link_name=None):
    """ Appends a pose to the given move_group goal, returns it appended
        Goals for now are for the same link TODO: let it be for different links"""
    if goal_to_append == None:
        rospy.logerr("append_pose_to_move_group_goal needs a goal!")
        return
    goal_to_append = MoveGroupGoal()
    goal_c = Constraints()
    position_c = PositionConstraint()
    position_c.header = goal_to_append.request.goal_constraints[0].header
    position_c.link_name = goal_to_append.request.goal_constraints[0].link_name
    position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
    position_c.constraint_region.primitive_poses.append(goal_pose)
    position_c.weight = 1.0
    goal_c.position_constraints.append(position_c)
    orientation_c = OrientationConstraint()
    orientation_c.header = goal_to_append.request.goal_constraints[0].header
    orientation_c.link_name = goal_to_append.request.goal_constraints[0].link_name
    orientation_c.orientation = goal_pose.orientation
    orientation_c.absolute_x_axis_tolerance = 0.01
    orientation_c.absolute_y_axis_tolerance = 0.01
    orientation_c.absolute_z_axis_tolerance = 0.01
    orientation_c.weight = 1.0
    goal_c.orientation_constraints.append(orientation_c)
    goal_to_append.request.goal_constraints.append(goal_c)
    return goal_to_append
    


if __name__=='__main__':
    rospy.init_node("pose_goal_test")
    
    rospy.loginfo("Connecting to move_group AS")
    moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    moveit_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.loginfo("Creating goal.")
    goal_point = Point(0.4, -0.2, 1.1)
    step = 0.06
    while True:
        goal_pose = Pose()
        goal_pose.position = goal_point
        goal_pose.orientation = Quaternion(w=1.0)
        moveit_goal = create_move_group_pose_goal(goal_pose, plan_only=False)
        #rospy.loginfo("Sending goal:\n" + str(moveit_goal))
        rospy.loginfo("Sending goal...")
        moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        moveit_ac.wait_for_result(rospy.Duration(0.5))
        moveit_result = moveit_ac.get_result()
        #rospy.loginfo("Got result:\n" + str(moveit_result))
        r = MoveGroupResult()
        if moveit_result != None and moveit_result.error_code.val != 1:
            rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
        elif moveit_result != None:
            rospy.loginfo("Goal achieved.")
        else:
            rospy.loginfo("Too slow, sending next goal")
#         goal_point.x += (random() - 0.5) / 10
#         goal_point.y += (random() - 0.5) / 10
#         goal_point.z += (random() - 0.5) / 10
        goal_point.z += step
        if goal_point.z >= 1.5 or goal_point.z <= 1.0:
            step *= -1
        rospy.loginfo("Setting new goal:\n " + str(goal_point))

