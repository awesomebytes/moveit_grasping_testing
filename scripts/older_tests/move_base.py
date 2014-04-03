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

#MOVE_BASE_TOPIC = '/mobile_base_controller/cmd_vel'
MOVE_BASE_TOPIC = '/nav_vel'

# rosrun tf static_transform_publisher 0.0 -1.0 1.0 0 0 0 /base_link /hydra_base 100


    

class RazerControl():

    def __init__(self):
        self.hydra_data_subs = rospy.Subscriber(HYDRA_DATA_TOPIC, Hydra, self.hydraDataCallback)
        self.pub_move_base = rospy.Publisher(MOVE_BASE_TOPIC, Twist)
        self.last_hydra_message = None
        self.read_message = False


    def hydraDataCallback(self, data):
        #rospy.loginfo("Received data from " + HYDRA_DATA_TOPIC)
        self.last_hydra_message = data
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
                if self.last_hydra_message.paddles[0].joy[0] != 0.0 or self.last_hydra_message.paddles[0].joy[1] != 0.0:
                    twist_goal = Twist()
                    twist_goal.linear.x = 1.0 * self.last_hydra_message.paddles[0].joy[1]
                    twist_goal.angular.z = 1.0 * self.last_hydra_message.paddles[0].joy[0] * -1.0
                    self.pub_move_base.publish(twist_goal)
                    # move base rotate left (neg)/ right(pos)
                    rospy.loginfo("Move base command sent")
                            
                    
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