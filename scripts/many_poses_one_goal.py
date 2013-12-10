#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 9 11:33:00 2013

@author: Sam Pfeiffer

THIS IS IGNORED!!!! >.<
"""
from __future__ import division
#from moveit_commander import MoveGroupCommander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PoseArray
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp, MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes, TrajectoryConstraints
import actionlib
from shape_msgs.msg import SolidPrimitive
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


    
    
def append_traj_to_move_group_goal(goal_to_append=None, goal_pose=Pose(), link_name=None):
    """ Appends a trajectory_point to the given move_group goal, returns it appended"""
    if goal_to_append == None:
        rospy.logerr("append_trajectory_point_to_move_group_goal needs a goal!")
        return
    #goal_to_append = MoveGroupGoal()
    #rospy.logwarn("goal_to_append is: \n" + str(goal_to_append))
    traj_c = TrajectoryConstraints()
    goal_c = Constraints()
    goal_c.name = "traj_constraint"
    # Position constraint
    position_c = PositionConstraint()
    position_c.header = goal_to_append.request.goal_constraints[0].position_constraints[0].header
    position_c.link_name = goal_to_append.request.goal_constraints[0].position_constraints[0].link_name if link_name == None else link_name
    position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
    position_c.constraint_region.primitive_poses.append(goal_pose)
    position_c.weight = 2.0
    goal_c.position_constraints.append(position_c)
    # Orientation constraint
    orientation_c = OrientationConstraint()
    orientation_c.header = goal_to_append.request.goal_constraints[0].position_constraints[0].header
    orientation_c.link_name = goal_to_append.request.goal_constraints[0].position_constraints[0].link_name if link_name == None else link_name
    orientation_c.orientation = goal_pose.orientation
    orientation_c.absolute_x_axis_tolerance = 0.01
    orientation_c.absolute_y_axis_tolerance = 0.01
    orientation_c.absolute_z_axis_tolerance = 0.01
    orientation_c.weight = 1.0
    goal_c.orientation_constraints.append(orientation_c)
    
    traj_c.constraints.append(goal_c)
    goal_to_append.request.trajectory_constraints = traj_c
    
    return goal_to_append
    
    
def interpolated_waypoints(initial_point=Point(), final_point=Point(), steps=5):
    """Returns a list of #steps points from initial_point to final_point interpolating the movement
    in a straight line"""
    if steps <= 0:
        rospy.logerr("steps must be >= 0")
        exit(0)
    print "initial_point: \n" + str(initial_point)
    print "final_point: \n" + str(final_point)
    stepx = (final_point.x - initial_point.x) / float(steps)
    stepy = (final_point.y - initial_point.y) / float(steps)
    stepz = (final_point.z - initial_point.z) / float(steps)
    pointlist = []
    for i in range(int(steps)):
        pointlist.append(Point(initial_point.x + (stepx * (i+1)), initial_point.y + (stepy * (i+1)), initial_point.z + (stepz * (i+1))))
    print "List of points is: \n" + str(pointlist)
    print "With steps: " + str(steps) + "; x=" + str(stepx) + "; y=" + str(stepy) + "; z=" + str(stepz)
    return pointlist
    

if __name__=='__main__':
    rospy.init_node("pose_goal_test")
    
    rospy.loginfo("Connecting to move_group AS")
    moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    moveit_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.loginfo("Creating goal.")
    goal_point = Point(0.4, -0.2, 1.1)
    goal_pose = Pose()
    goal_pose.position = goal_point
    goal_pose.orientation = Quaternion(w=1.0)
    moveit_goal = create_move_group_pose_goal(goal_pose, plan_only=False)
    
    on_starting_pose = False
    while not on_starting_pose:
        rospy.loginfo("Sending Starting position goal...")
        moveit_ac.send_goal(moveit_goal)
        rospy.loginfo("Waiting for result...")
        moveit_ac.wait_for_result(rospy.Duration(6.0))
        moveit_result = moveit_ac.get_result()
        r = MoveGroupResult()
        if moveit_result != None and moveit_result.error_code.val != 1:
            rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
        elif moveit_result != None:
            rospy.loginfo("Goal achieved.")
            on_starting_pose = True
        else:
            on_starting_pose = True
            rospy.loginfo("Another error")
    
    rospy.loginfo("Creating final goal and trajectory points")
    waypoints_pa = PoseArray()
    starting_point = Point(goal_point.x, goal_point.y, goal_point.z)
    goal_point = Point(0.4, 0.1, 1.6)
#     another_goal = Point(0.4, 0.0, 1.1)
    list_waypoints = interpolated_waypoints(starting_point, goal_point, 30)
#     list_waypoints2 = interpolated_waypoints(goal_point, another_goal, 5)
#     list_waypoints.extend(list_waypoints2)
    
    goal_pose = Pose()
    goal_pose.position = goal_point
    goal_pose.orientation = Quaternion(w=1.0)
    waypoints_pa.header.frame_id = 'base_link'
    waypoints_pa.header.stamp = rospy.Time.now()
    waypoints_pa.poses.append(Pose(position=Point(starting_point.x, starting_point.y, starting_point.z),orientation=Quaternion(w=1.0)))
    waypoints_pa.poses.append(goal_pose)
    moveit_goal2 = create_move_group_pose_goal(goal_pose, plan_only=False)
    for curr_point in list_waypoints:
        goal_pose = Pose()
        goal_pose.position = curr_point
        goal_pose.orientation = Quaternion(w=1.0)
        #rospy.loginfo("Setting new traj point:\n " + str(starting_point))
        moveit_goal2 = append_traj_to_move_group_goal(moveit_goal2, goal_pose)
        waypoints_pa.poses.append(goal_pose)
    

    rospy.loginfo("Sending goal...")
    moveit_ac.send_goal(moveit_goal2)
    rospy.loginfo("Waiting for result...")
    moveit_ac.wait_for_result(rospy.Duration(6.0))
    moveit_result = moveit_ac.get_result()
    #rospy.loginfo("Got result:\n" + str(moveit_result))
    r = MoveGroupResult()
    if moveit_result != None and moveit_result.error_code.val != 1:
        rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
    elif moveit_result != None:
        rospy.loginfo("Goal achieved.")
    else:
        rospy.loginfo("Another error")

    rospy.loginfo("Publishing waypoints in PoseArray /waypoints")
    pub_waypoints = rospy.Publisher('/waypoints', PoseArray, latch=True)
    while True:
        pub_waypoints.publish(waypoints_pa)
        rospy.sleep(0.1)
