#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 10 11:54:00 2013

@author: Sam Pfeiffer
"""

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState



HEAD_AS = '/head_controller/follow_joint_trajectory'

class controlHeadOculus():

    def __init__(self):
        rospy.Subscriber('/oculus_pose', PoseStamped, self.oculus_pose_callback)
        self.last_oculus_msg = None
        rospy.loginfo("Connecting with head AS")
        self.head_as = actionlib.SimpleActionClient(HEAD_AS, FollowJointTrajectoryAction)
        self.head_as.wait_for_server()
        rospy.loginfo("Connected!")
        rospy.loginfo("Waiting for oculus pose...")
        while self.last_oculus_msg == None:
            rospy.sleep(0.1)
        rospy.loginfo("Done! Starting head controlling.")
        self.sendCommandsHead()
        
    def oculus_pose_callback(self, data):
        self.last_oculus_msg = data
        
    def createHeadGoal(self):
        (roll, pitch, yaw) = euler_from_quaternion([self.last_oculus_msg.pose.orientation.x,
                                                    self.last_oculus_msg.pose.orientation.y,
                                                    self.last_oculus_msg.pose.orientation.z,
                                                    self.last_oculus_msg.pose.orientation.w])
        rospy.loginfo("roll, pitch, yaw: ")
        rospy.loginfo(str(roll) + " " + str(pitch) + " " + str(yaw))
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names.append('head_1_joint') # 1 positivo izquierda, -1 derecha
        head_goal.trajectory.joint_names.append('head_2_joint') # -1 arriba, 1 abajo
        jtp = JointTrajectoryPoint()        
        jtp.positions.append(yaw *-1)
        jtp.positions.append(pitch *-1)
#         jtp.positions.append((yaw *-1) + 1.5707963267948966) # + 180 deg
#         jtp.positions.append((roll + 1.5707963267948966) *-1) # + 180 deg because of torso2
        jtp.velocities.append(0.0)
        jtp.velocities.append(0.0)
        rospy.loginfo("Sending: " + str(jtp.positions))
        #jtp.time_from_start.secs = 1
        jtp.time_from_start.nsecs = 300
        head_goal.trajectory.points.append(jtp)
        return head_goal
        
        
    def sendCommandsHead(self):
        while True:
            head_goal = self.createHeadGoal()
            self.head_as.send_goal(head_goal)
            self.head_as.wait_for_result()
            rospy.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('oculus_move_head')
    cho = controlHeadOculus()

    
