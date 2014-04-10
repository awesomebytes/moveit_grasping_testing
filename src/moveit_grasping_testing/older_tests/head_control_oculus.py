#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 10 11:54:00 2013

@author: Sam Pfeiffer
"""

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose, PointStamped, Vector3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, PointHeadAction, PointHeadActionGoal, PointHeadGoal
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header



HEAD_AS = '/head_controller/follow_joint_trajectory'
POINT_HEAD_AS = '/head_controller/point_head_action'

class controlHeadOculus():

    def __init__(self):
        rospy.Subscriber('/oculus_pose', PoseStamped, self.oculus_pose_callback)
        self.last_oculus_msg = None
        self.current_oculus_msg = None
#         rospy.loginfo("Connecting with head AS")
#         self.head_as = actionlib.SimpleActionClient(HEAD_AS, FollowJointTrajectoryAction)
#         self.head_as.wait_for_server()
#         rospy.loginfo("Connected!")
        rospy.loginfo("Connecting with point head AS")
        self.head_point_as = actionlib.SimpleActionClient(POINT_HEAD_AS, PointHeadAction)
        self.head_point_as.wait_for_server()
        rospy.loginfo("Connected!")        
        
        
        rospy.loginfo("Waiting for oculus pose...")
        while self.last_oculus_msg == None:
            rospy.sleep(0.1)
        rospy.loginfo("Done! Starting head controlling.")
        #self.sendCommandsHead()
        self.sendCommandsPointHead()
        
    def oculus_pose_callback(self, data):
        self.last_oculus_msg = self.current_oculus_msg
        self.current_oculus_msg = data

        
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
        jtp.time_from_start.nsecs = 100
        head_goal.trajectory.points.append(jtp)
        head_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        return head_goal
        
        
    def createPointHeadActionGoal(self):
        phag = PointHeadGoal()
        phag.pointing_frame = 'stereo_link'
        phag.pointing_axis = Vector3(1.0, 0.0, 0.0)
        phag.min_duration = rospy.Duration(1.0)
        phag.target.header = Header(frame_id='oculus', stamp=rospy.Time.now())
        phag.target.point = Point(1.0, 0.0, 0.0)
        return phag
    
    def sendCommandsPointHead(self):
        while True:
            head_point_goal = self.createPointHeadActionGoal()
            self.head_point_as.send_goal(head_point_goal)
            self.head_point_as.wait_for_result(rospy.Duration(0.0))
        
    def sendCommandsHead(self):
        while True:
            head_goal = self.createHeadGoal()
            self.head_as.send_goal(head_goal)
            self.head_as.wait_for_result(rospy.Duration(0.0))
            #rospy.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('oculus_move_head')
    cho = controlHeadOculus()

    
