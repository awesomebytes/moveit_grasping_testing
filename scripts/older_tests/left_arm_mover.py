#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thursday Aug 8 10:10:55 2013

@author: sampfeiffer
"""
import rospy
#import sys
import actionlib
import rosbag
from datetime import datetime
from razer_hydra.msg import Hydra
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose, Twist
#from nav_msgs.msg import Path
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import Grasp, MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


HYDRA_DATA_TOPIC = '/hydra_calib'


LEFT_HAND_POSESTAMPED_TOPIC = '/teleop_left_hand_pose'
LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_left_hand_pose_reference'

#TODO: Use head orientation topic of oculus to move head

#TODO: separate subscribers of razer topic for hands, arms, etc... so we can have simultaneous goals

LEFT_HAND_INITIAL_POINT = Point(x=0.6, y=0.2, z=1.1)


# rosrun tf static_transform_publisher 0.0 -1.0 1.0 0 0 0 /base_link /hydra_base 100


    

class RazerControl():

    def __init__(self):
        self.pub_left_hand_pose = rospy.Publisher(LEFT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose_reference = rospy.Publisher(LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.hydra_data_subs = rospy.Subscriber(HYDRA_DATA_TOPIC, Hydra, self.hydraDataCallback)

        rospy.loginfo("Connecting to left arm AS")
        self.moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        self.moveit_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        self.last_hydra_message = None
        self.tmp_pose_left = PoseStamped()
        self.read_message = False


    def create_move_group_pose_goal(self, goal_pose=Pose(), group="left_arm", end_link_name="arm_left_tool_link", plan_only=True):
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
        moveit_goal.request.num_planning_attempts = 3
        moveit_goal.request.allowed_planning_time = 1.0
        moveit_goal.planning_options.plan_only = plan_only
        moveit_goal.planning_options.planning_scene_diff.is_diff = True
        moveit_goal.request.group_name = group
        
        return moveit_goal



    def hydraDataCallback(self, data):
        #rospy.loginfo("Received data from " + HYDRA_DATA_TOPIC)
        self.last_hydra_message = data
        self.tmp_pose_left = PoseStamped()
        self.tmp_pose_left.header.frame_id = 'base_link'
        self.tmp_pose_left.header.stamp = rospy.Time.now()
        self.tmp_pose_left.pose.position.x = self.last_hydra_message.paddles[0].transform.translation.x
        self.tmp_pose_left.pose.position.y = self.last_hydra_message.paddles[0].transform.translation.y
        self.tmp_pose_left.pose.position.z = self.last_hydra_message.paddles[0].transform.translation.z
        self.tmp_pose_left.pose.position.x += LEFT_HAND_INITIAL_POINT.x
        self.tmp_pose_left.pose.position.y += LEFT_HAND_INITIAL_POINT.y
        self.tmp_pose_left.pose.position.z += LEFT_HAND_INITIAL_POINT.z
        
        self.tmp_pose_left.pose.orientation = self.last_hydra_message.paddles[0].transform.rotation
        if self.last_hydra_message.paddles[0].buttons[0] == True:
            self.pub_left_hand_pose.publish(self.tmp_pose_left)
            
        self.pub_left_hand_pose_reference.publish(self.tmp_pose_left)
        self.read_message = False

    def run(self):
        rospy.loginfo("Press LB / RB to send the current pose")
        
        while self.last_hydra_message == None:
            rospy.sleep(0.1)
            
        rospy.loginfo("Got the first data of the razer... Now we can do stuff")
        
        sleep_rate=0.05 # check at 20Hz
        counter = 0
        while True:
            counter += 1
            #rospy.loginfo("Loop #" + str(counter))
            if not self.read_message:
                self.read_message = True
                    
                if self.last_hydra_message.paddles[0].buttons[0] == True:
                    # send curr right paddle pos to move_group left
                    rospy.loginfo("Sending current left hand position & orientation")
                    moveit_goal = self.create_move_group_pose_goal(self.tmp_pose_left.pose, group="left_arm", end_link_name="arm_left_tool_link", plan_only=False)
#                     self.right_arm_mgc.set_pose_target(self.tmp_pose_right)
#                     self.right_arm_mgc.go(wait=False)

                    rospy.loginfo("Sending goal...")
                    self.moveit_ac.send_goal(moveit_goal)
                    rospy.loginfo("Waiting for result...")
                    self.moveit_ac.wait_for_result(rospy.Duration(1.0))
                    moveit_result = self.moveit_ac.get_result()
                    #rospy.loginfo("Got result:\n" + str(moveit_result))
                    r = MoveGroupResult()
                    if moveit_result != None and moveit_result.error_code.val != 1:
                        rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
                    elif moveit_result != None:
                        rospy.loginfo("Goal achieved.")
                    else:
                        rospy.loginfo("Too slow, sending next goal")
                    

            rospy.sleep(sleep_rate)


if __name__ == '__main__':
    rospy.init_node('hydra_left_arm')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    node = RazerControl()
    node.run()

    rospy.spin()