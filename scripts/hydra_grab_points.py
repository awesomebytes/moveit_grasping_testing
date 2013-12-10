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
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

HYDRA_DATA_TOPIC = '/hydra_calib'
HAND_RIGHT_AS = '/right_hand_controller/follow_joint_trajectory'
HAND_LEFT_AS = '/left_hand_controller/follow_joint_trajectory'

RIGHT_HAND_POSESTAMPED_TOPIC = '/teleop_right_hand_pose'
LEFT_HAND_POSESTAMPED_TOPIC = '/teleop_left_hand_pose'
RIGHT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_right_hand_pose_reference'
LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_left_hand_pose_reference'

#TODO: Use head orientation topic of oculus to move head

#TODO: separate subscribers of razer topic for hands, arms, etc... so we can have simultaneous goals

RIGHT_HAND_INITIAL_POINT = Point(x=0.6, y=-0.2, z=1.1)
LEFT_HAND_INITIAL_POINT = Point(x=0.6, y=0.2, z=1.1)

MOVE_BASE_TOPIC = '/mobile_base_controller/cmd_vel'

# rosrun tf static_transform_publisher 0.0 -1.0 1.0 0 0 0 /base_link /hydra_base 100


    

class RazerControl():

    def __init__(self):
        self.pub_right_hand_pose = rospy.Publisher(RIGHT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_right_hand_pose_reference = rospy.Publisher(RIGHT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose = rospy.Publisher(LEFT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose_reference = rospy.Publisher(LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.hydra_data_subs = rospy.Subscriber(HYDRA_DATA_TOPIC, Hydra, self.hydraDataCallback)
        self.pub_move_base = rospy.Publisher(MOVE_BASE_TOPIC, Twist)
        self.subs = rospy.Subscriber('/joint_states', JointState, self.getJointStates)
        self.current_joint_states = None
        rospy.loginfo("Getting first joint_states")
        while self.current_joint_states == None:
            rospy.sleep(0.1)
        rospy.loginfo("Gotten!")
        rospy.loginfo("Connecting with right hand AS")
        self.right_hand_as = actionlib.SimpleActionClient(HAND_RIGHT_AS, FollowJointTrajectoryAction)
        self.right_hand_as.wait_for_server()
        rospy.loginfo("Connecting with left hand AS")
        self.left_hand_as = actionlib.SimpleActionClient(HAND_LEFT_AS, FollowJointTrajectoryAction)
        self.left_hand_as.wait_for_server()
        rospy.loginfo("Starting up move group commander for right, left, torso and head... (slow)")
        self.right_arm_mgc = MoveGroupCommander("right_arm")
        self.right_arm_mgc.set_pose_reference_frame('base_link')
        self.left_arm_mgc = MoveGroupCommander("left_arm")
        self.left_arm_mgc.set_pose_reference_frame('base_link')
        self.torso_mgc = MoveGroupCommander("right_arm_torso")
        self.torso_mgc.set_pose_reference_frame('base_link')
        self.head_mgc = MoveGroupCommander("head")
        self.head_mgc.set_pose_reference_frame('base_link')
        self.last_hydra_message = None
        self.tmp_pose_right = PoseStamped()
        self.tmp_pose_left = PoseStamped()
        self.read_message = False

    def getJointStates(self, data):
        self.current_joint_states = data
        
    def create_hand_goal(self, hand_side="right", hand_pose="closed", values=0.0):
        """Returns the hand goal to send
        possible poses: closed, open, intermediate"""
        hand_goal = FollowJointTrajectoryGoal()
        hand_goal.trajectory.joint_names.append('hand_'+ hand_side +'_thumb_joint')
        hand_goal.trajectory.joint_names.append('hand_'+ hand_side +'_middle_joint')
        hand_goal.trajectory.joint_names.append('hand_'+ hand_side +'_index_joint')
        jtp = JointTrajectoryPoint()
        
        joint_list = ['hand_'+ hand_side +'_thumb_joint',
                      'hand_'+ hand_side +'_middle_joint',
                      'hand_'+ hand_side +'_index_joint']
        ids_list = []
        values_list = []
        rospy.loginfo("current_joint_state is:\n" + str(self.current_joint_states))
        for joint in joint_list:
            idx_in_message = self.current_joint_states.name.index(joint)
            ids_list.append(idx_in_message)
            values_list.append(self.current_joint_states.position[idx_in_message])
        
        if hand_pose == "closed":
            jtp.positions.append(2.0)
            jtp.positions.append(values_list[1]) # TODO: read values and keep them
            jtp.positions.append(values_list[2]) # TODO: read values and keep them
        elif hand_pose == "open":
            jtp.positions.append(0.0)
            jtp.positions.append(values_list[1]) # TODO: read values and keep them
            jtp.positions.append(values_list[2]) # TODO: read values and keep them
        elif hand_pose == "intermediate":
            jtp.positions.append(values_list[0]) # TODO: read values and keep them
            jtp.positions.append(values) 
            jtp.positions.append(values)
        jtp.velocities.append(0.0)
        jtp.velocities.append(0.0)
        jtp.velocities.append(0.0)
        jtp.time_from_start.secs = 2
        hand_goal.trajectory.points.append(jtp)
        return hand_goal

    def hydraDataCallback(self, data):
        #rospy.loginfo("Received data from " + HYDRA_DATA_TOPIC)
        self.last_hydra_message = data
        self.tmp_pose_right = PoseStamped()
        self.tmp_pose_right.header.frame_id = 'base_link'
        self.tmp_pose_right.header.stamp = rospy.Time.now()
        self.tmp_pose_right.pose.position.x = self.last_hydra_message.paddles[1].transform.translation.x
        self.tmp_pose_right.pose.position.y = self.last_hydra_message.paddles[1].transform.translation.y
        self.tmp_pose_right.pose.position.z = self.last_hydra_message.paddles[1].transform.translation.z
        self.tmp_pose_right.pose.position.x += RIGHT_HAND_INITIAL_POINT.x
        self.tmp_pose_right.pose.position.y += RIGHT_HAND_INITIAL_POINT.y
        self.tmp_pose_right.pose.position.z += RIGHT_HAND_INITIAL_POINT.z
        self.tmp_pose_right.pose.orientation = self.last_hydra_message.paddles[1].transform.rotation
        
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
        if self.last_hydra_message.paddles[1].buttons[0] == True:
            self.pub_right_hand_pose.publish(self.tmp_pose_right)
        if self.last_hydra_message.paddles[0].buttons[0] == True:
            self.pub_left_hand_pose.publish(self.tmp_pose_left)
            
        self.pub_right_hand_pose_reference.publish(self.tmp_pose_right)
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
            rospy.loginfo("Loop #" + str(counter))
            if not self.read_message:
                self.read_message = True
                if self.last_hydra_message.paddles[1].buttons[0] == True:
                    # send curr left paddle pos to move_group right
                    rospy.loginfo("sending curr right hand")
                    self.right_arm_mgc.set_pose_target(self.tmp_pose_right)
                    self.right_arm_mgc.go(wait=False)
                    
                if self.last_hydra_message.paddles[0].buttons[0] == True:
                    # send curr right paddle pos to move_group left
                    rospy.loginfo("sending curr left hand")
                    self.left_arm_mgc.set_pose_target(self.tmp_pose_left)
                    self.left_arm_mgc.go(wait=False)
                    
                if self.last_hydra_message.paddles[1].trigger > 0.0:
                    # send goal right hand close proportional to trigger value (2.0 max?)
                    rospy.loginfo("Closing right hand to value: " + str(self.last_hydra_message.paddles[1].trigger * 2.0))
                    right_hand_goal = self.create_hand_goal(hand_side="right", hand_pose="intermediate", values=self.last_hydra_message.paddles[1].trigger * 2.0)
                    self.right_hand_as.send_goal(right_hand_goal)
                    
                if self.last_hydra_message.paddles[0].trigger > 0.0:
                    # send goal left hand close proportional to trigger value (2.0 max?)
                    rospy.loginfo("Closing left hand to value: " + str(self.last_hydra_message.paddles[0].trigger * 2.0))
                    left_hand_goal = self.create_hand_goal(hand_side="left", hand_pose="intermediate", values=self.last_hydra_message.paddles[0].trigger * 2.0)
                    self.left_hand_as.send_goal(left_hand_goal)
                    
                if self.last_hydra_message.paddles[1].joy[0] != 0.0:
                    # send torso rotation left(neg)/right (pos)
                    rospy.loginfo("Rotation torso")
                    curr_joint_val = self.torso_mgc.get_current_joint_values()
                    self.torso_mgc.set_joint_value_target("torso_1_joint", curr_joint_val[0] + (self.last_hydra_message.paddles[1].joy[0] * 0.1 * -1))
                    self.torso_mgc.go(wait=True)
                    rospy.loginfo("Rotation torso sent!")
                    
                if self.last_hydra_message.paddles[1].joy[1] != 0.0:
                    # send torso inclination front(pos)/back(neg)
                    rospy.loginfo("Inclination torso")
                    curr_joint_val = self.torso_mgc.get_current_joint_values()
                    self.torso_mgc.set_joint_value_target("torso_2_joint", curr_joint_val[1] + (self.last_hydra_message.paddles[1].joy[1] * 0.1))
                    self.torso_mgc.go(wait=True)
                    rospy.loginfo("Inclination torso sent!")
                    
                if self.last_hydra_message.paddles[0].joy[0] != 0.0 or self.last_hydra_message.paddles[0].joy[1] != 0.0:
                    twist_goal = Twist()
                    twist_goal.linear.x = 1.0 * self.last_hydra_message.paddles[0].joy[1]
                    twist_goal.angular.z = 1.0 * self.last_hydra_message.paddles[0].joy[0] * -1.0
                    self.pub_move_base.publish(twist_goal)
                    # move base rotate left (neg)/ right(pos)
                    rospy.loginfo("Move base")
                            
                    
                if self.last_hydra_message.paddles[1].buttons[3] == True:
                    # thumb up
                    rospy.loginfo("Right thumb up")
                    right_thumb_up = self.create_hand_goal(hand_side="right", hand_pose="open")
                    self.right_hand_as.send_goal(right_thumb_up)
                  
                if self.last_hydra_message.paddles[0].buttons[3] == True:
                    # thumb up
                    rospy.loginfo("Left thumb up") 
                    left_thumb_up = self.create_hand_goal(hand_side="left", hand_pose="open")
                    self.left_hand_as.send_goal(left_thumb_up)
                
                if self.last_hydra_message.paddles[1].buttons[1] == True:
                    # thumb down
                    rospy.loginfo("Right thumb down")
                    right_thumb_up = self.create_hand_goal(hand_side="right", hand_pose="closed")
                    self.right_hand_as.send_goal(right_thumb_up)
     
                if self.last_hydra_message.paddles[0].buttons[1] == True:
                    # thumb down
                    rospy.loginfo("Left thumb down")
                    left_thumb_up = self.create_hand_goal(hand_side="left", hand_pose="closed")
                    self.left_hand_as.send_goal(left_thumb_up)            
                
            rospy.sleep(sleep_rate)


if __name__ == '__main__':
    rospy.init_node('hydra_info')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    node = RazerControl()
    node.run()

    rospy.spin()