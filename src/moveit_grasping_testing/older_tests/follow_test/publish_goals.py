#!/usr/bin/python

import rospy
import geometry_msgs.msg as GM
TO_FOLLOW_POSE_TOPIC = "pipol_pose"
import copy
import random


if __name__ == '__main__':
    rospy.init_node("testing_follow_publisher")
    pub = rospy.Publisher(TO_FOLLOW_POSE_TOPIC, GM.PoseStamped)
    initialpose = GM.PoseStamped()
    initialpose.header.frame_id = "odom"
    initialpose.pose.position.x = 1.0
    initialpose.pose.orientation.w = 1.0
    last_goal = initialpose
    while not rospy.is_shutdown():
        goal = copy.deepcopy(last_goal)
        goal.pose.position.x += 0.35
        goal.pose.position.y += 0.25 * int(random.random() + 0.5 / 2) * -1  # randomly going left or right
        pub.publish(goal)
        last_goal = goal
        rospy.sleep(2)

