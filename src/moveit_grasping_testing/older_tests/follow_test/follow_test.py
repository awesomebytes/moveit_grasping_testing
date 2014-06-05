#!/usr/bin/python

import rospy
import geometry_msgs.msg as GM
from follow_planner import FollowMe

TO_FOLLOW_POSE_TOPIC = "pipol_pose"


def callback_f(data):
    rospy.loginfo("Got callback: \n" + str(data))
    fm.update(data)

if __name__ == '__main__':
    rospy.init_node("testing_follow_nav")
    sub = rospy.Subscriber(TO_FOLLOW_POSE_TOPIC, GM.PoseStamped, callback_f)

    rospy.loginfo("Creating FollowMe instance")
    fm = FollowMe()
    rospy.loginfo("Starting FollowMe")
    fm.start()

    rospy.loginfo("Doing nothing for a while...")
    rospy.sleep(15)
    rospy.loginfo("Stopping follow")
    fm.stop()
