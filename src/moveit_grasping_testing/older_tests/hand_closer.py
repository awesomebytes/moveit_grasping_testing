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

#TODO: Use head orientation topic of oculus to move head

#TODO: separate subscribers of razer topic for hands, arms, etc... so we can have simultaneous goals



# rosrun tf static_transform_publisher 0.0 -1.0 1.0 0 0 0 /base_link /hydra_base 100


    

class RazerControl():

    def __init__(self):
        self.hydra_data_subs = rospy.Subscriber(HYDRA_DATA_TOPIC, Hydra, self.hydraDataCallback)
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

        self.last_hydra_message = None

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
        #rospy.loginfo("current_joint_state is:\n" + str(self.current_joint_states))
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
        self.read_message = False

    def run(self):
        rospy.loginfo("Press LB / RB to send the current pose")
        
        while self.last_hydra_message == None:
            rospy.sleep(0.1)
            
        rospy.loginfo("Got the first data of the razer... Now we can do stuff")
        
        sleep_rate=0.1 # check at 20Hz
        counter = 0
        while True:
            counter += 1
            #rospy.loginfo("Loop #" + str(counter))
            if not self.read_message:
                self.read_message = True
                    
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
    rospy.init_node('hydra_hand_closer')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    node = RazerControl()
    node.run()

    rospy.spin()