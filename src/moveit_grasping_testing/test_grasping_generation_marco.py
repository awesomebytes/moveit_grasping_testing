#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Based on the version made:
Created on April 2 15:20:00 2014
This one is created 21/05/15
@author: Sam Pfeiffer
"""

# , roscpp_initialize, roscpp_shutdown
from moveit_commander import RobotCommander, PlanningSceneInterface
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, GripperTranslation, MoveItErrorCodes
import copy
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GenerateGraspsResult, GraspGeneratorOptions
from tf import transformations
from math import radians, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_about_axis, unit_vector, quaternion_multiply
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Pose, Quaternion, PoseArray, Vector3, Point
from std_msgs.msg import ColorRGBA, Header
import math
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import copy


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

# http://stackoverflow.com/questions/17044296/quaternion-rotation-without-euler-angles


def quaternion_from_vectors(v0, v1):
    if type(v0) == Point():
        v0 = [v0.x, v0.y, v0.z]
    if type(v1) == Point():
        v1 = [v1.x, v1.y, v1.z]

    v0 = normalize(v0)
    v1 = normalize(v1)
#     inline QuaternionT<T> QuaternionT<T>::CreateFromVectors(const Vector3<T>& v0, const Vector3<T>& v1)
# {
#
#     Vector3<T> c = v0.Cross(v1);
    c = np.cross(v0, v1)
#     T d = v0.Dot(v1);
    d = np.dot(v0, v1)
#     T s = std::sqrt((1 + d) * 2);
    try:
        s = math.sqrt((1.0 + d) * 2)
    except ValueError:
        s = 0.0
    if s == 0.0:
        # print "s == 0.0, we cant compute"
        return [0.0, 0.0, 0.0, 1.0]
#
#     QuaternionT<T> q;
    q = [0.0, 0.0, 0.0, 0.0]
#     q.x = c.x / s;
    q[0] = c[0] / s
#     q.y = c.y / s;
    q[1] = c[1] / s
#     q.z = c.z / s;
    q[2] = c[2] / s
#     q.w = s / 2.0f;
    q[3] = s / 2.0
#     return q;
    return q
# }


def createPickupGoal(group="right_arm_torso", target="part", grasp_pose=PoseStamped(), possible_grasps=[]):
    """ Create a PickupGoal with the provided data"""
    pug = PickupGoal()  # haha goal is a dog
    pug.target_name = target
    pug.group_name = group
    #pug.end_effector = "right_eef"
    # pug.end_effector = 'right_eef' # should check if i can put other links
    # here... i dont think so :S
    pug.possible_grasps.extend(possible_grasps)
    # Compare this to the... minute we had before!!
    pug.allowed_planning_time = 15.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 1  # 10
    # pug.allowed_touch_objects.append("all")
    # pug.attached_object_touch_links.append("all") #looks bad XD

    return pug


def filter_poses(sphere_poses, object_pose):
    # TODO: first just filter poses that look from a wrong direction
    # from there i'll learn how to filter
    # then generate approach vectors taking that into account
    # return sphere_poses  # for the sake of keep testing retunr the same for
    # now
    new_list = []
    for pose in sphere_poses:
        # if pose is further away than object, ditch it
        if pose.position.x > object_pose.pose.position.x:
            continue
        # if pose if under the object, ditch it
        if pose.position.z < object_pose.pose.position.z:
            continue
        else:
            new_list.append(pose)
    return new_list


def sort_by_height(sphere_poses):
    # We prefer to grasp from top to be safer
    newlist = sorted(
        sphere_poses, key=lambda item: item.position.z, reverse=False)
    sorted_list = newlist
    return sorted_list


def retrieveGrasps(pose, width=0.04):
    # Data we will need for fulfilling later:
    object_pose = pose
    object_pose.header.frame_id = '/base_footprint'
    # object_pose.pose.position = Point(0.0, 0.0, 0.0)
    # object_pose.pose.orientation = Quaternion(0,0,0,1)
    time_for_motions = 2.0  # seconds
    min_motor_pos = 0.0
    max_motor_pos = 5.0
    thumb_max = 5.0
    joint_names = ['hand_index_joint', 'hand_mrl_joint', 'hand_thumb_joint']
    grasping_frame = 'hand_grasping_frame'  # 'arm_tool_link'
    approach_frame = 'arm_tool_link'
    min_distances = 0.0
    desired_distances = 0.15  # 1.0
    vector_front = Vector3Stamped()
    vector_front.header.frame_id = approach_frame
    vector_front.vector.x = -1.0
    vector_front.vector.y = 0.0
    vector_front.vector.z = 0.0
    vector_back = Vector3Stamped()
    vector_back.header.frame_id = approach_frame
    vector_back.vector.x = 1.0
    vector_back.vector.y = 0.0
    vector_back.vector.z = 0.0
    vector_up = Vector3Stamped()
    vector_up.header.frame_id = approach_frame
    vector_up.vector.x = 0.0
    vector_up.vector.y = 0.0
    vector_up.vector.z = 1.0
    vector_down = Vector3Stamped()
    vector_down.header.frame_id = approach_frame
    vector_down.vector.x = 0.0
    vector_down.vector.y = 0.0
    vector_down.vector.z = -1.0

    # Compute all the points of the sphere with step of 1 deg
    # http://math.stackexchange.com/questions/264686/how-to-find-the-3d-coordinates-on-a-celestial-spheres-surface
    radius = desired_distances
    ori_x = 0.0  # object_pose.pose.position.x
    ori_y = 0.0  # object_pose.pose.position.y
    ori_z = 0.0  # object_pose.pose.position.z
    sphere_poses = []
    rotated_q = quaternion_from_euler(0.0, 0.0, math.radians(180))
    degree_step = 10
    print "Creating points"
    for altitude in range(0, 360, degree_step):  # altitude is yaw
        altitude = math.radians(altitude)
        for azimuth in range(0, 360, degree_step):  # azimuth is pitch
            azimuth = math.radians(azimuth)
            # This gets all the positions correctly
            x = ori_x + radius * math.cos(azimuth) * math.cos(altitude)
            y = ori_y + radius * math.sin(altitude)
            z = ori_z + radius * math.sin(azimuth) * math.cos(altitude)
            # this gets all the vectors pointing outside of the center
            # quaternion as x y z w
            q = quaternion_from_vectors([radius, 0.0, 0.0], [x, y, z])
            #q = quaternion_from_vectors([ori_x, ori_y, ori_z], [x,y,z])
            # We invert the orientations to look inwards by multiplying with a
            # quaternion 180deg rotation on yaw
            q = quaternion_multiply(q, rotated_q)

            # In the case of tiago the gripper is rotated over roll
            # 90deg over the arm_tool_link so we need to fix up for that rotation
            # this must be computed generically
            # getting once the transform in between the
            # gripper link and the parent
            # fix_tool_to_gripper_rotation_q = quaternion_from_euler(
            #     math.radians(-90.0), 0.0, 0.0)
            # q = quaternion_multiply(q, fix_tool_to_gripper_rotation_q)

            # We actually want roll to be always 0.0
            roll, pitch, yaw = euler_from_quaternion(q)
            q = quaternion_from_euler(math.radians(-90.0), pitch, yaw)


            x += object_pose.pose.position.x
            y += object_pose.pose.position.y
            z += object_pose.pose.position.z
            current_pose = Pose(
                Point(x, y, z), Quaternion(*q))
            # current_pose = Pose(Point(x,y,z),
            # Quaternion(*quaternion_from_euler(0.0, -azimuth, altitude))) #
            # this worked once
            sphere_poses.append(current_pose)
    print "Done."

    poses_pub = rospy.Publisher('/sphere_poses', PoseArray, latch=True)

    sphere_poses = filter_poses(sphere_poses, object_pose)
    sphere_poses = sort_by_height(sphere_poses)

    pa = PoseArray()
    pa.header.frame_id = 'base_footprint'
    for pose in sphere_poses:
        pa.poses.append(pose)

    poses_pub.publish(pa)

    # rospy.sleep(3.0)
    # exit(0)

    # Now create all the grasps
    idx = 0
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = grasping_frame
    pre_grasp_posture.joint_names = joint_names
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = [min_motor_pos, min_motor_pos]
    jtpoint.time_from_start = rospy.Duration(time_for_motions)
    pre_grasp_posture.points.append(jtpoint)

    grasp_posture = copy.deepcopy(pre_grasp_posture)
    jtpoint2 = JointTrajectoryPoint()
    jtpoint2.positions = [max_motor_pos, max_motor_pos]
    jtpoint2.time_from_start = rospy.Duration(time_for_motions * 2)
    grasp_posture.points.append(jtpoint2)

    grasps = []
    for pose in sphere_poses:
        g = Grasp()
        g.id = '' + str(idx)
        # Position of the joints pre_grasp e.g.: open gripper
        g.pre_grasp_posture = pre_grasp_posture
        # Position of the joints in grasp e.g.: closed gripper
        g.grasp_posture = grasp_posture
        header = Header()
        header.frame_id = 'base_footprint'
        # The pose of the gripper, should be based on the pose of the object
        g.grasp_pose = PoseStamped(header, pose)
        g.grasp_quality = 0.1
        g.pre_grasp_approach = GripperTranslation()
        # The direction is in relation to the grasping frame, I think
        g.pre_grasp_approach.direction = vector_back
        g.pre_grasp_approach.direction.header.frame_id = approach_frame
        g.pre_grasp_approach.desired_distance = desired_distances
        g.pre_grasp_approach.min_distance = min_distances
        g.post_grasp_retreat = GripperTranslation()
        g.post_grasp_retreat.direction = vector_front
        g.post_grasp_retreat.direction.header.frame_id = approach_frame
        g.post_grasp_retreat.desired_distance = desired_distances
        g.post_grasp_retreat.min_distance = min_distances
        # g.post_place_retreat = vector_back
        # g.post_place_retreat.direction = vector_back
        # g.post_place_retreat.direction.header = grasping_frame
        # g.post_place_retreat.desired_distance = desired_distances
        # g.post_place_retreat.min_distance = min_distances
        g.max_contact_force = 0.
        g.allowed_touch_objects = []
        idx += 1

        grasps.append(g)

        # g2 = copy.deepcopy(g)
        # g2.id = '' + str(idx)
        # g2.pre_grasp_approach.direction = vector_up
        # g2.post_grasp_retreat.direction = vector_down
        # grasps.append(g2)
        # idx += 1

    # print "Grasps is:"
    # print grasps[:5]
    return grasps


if __name__ == '__main__':
    import sys
    rospy.init_node("pick_object_test")
    args = rospy.myargv(sys.argv)
    if len(args) > 1:
        obj_x = float(args[1])
        obj_y = float(args[2])
        obj_z = float(args[3])
    else:
        obj_x = 0.6
        obj_y = 0.35
        obj_z = 0.7

    rospy.loginfo("Connecting to pickup AS")
    pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")

    reset_part = True

    scene = PlanningSceneInterface()

    rospy.sleep(1)

    rospy.loginfo("Cleaning world objects")
    # clean the scene
    scene.remove_world_object("table")
    if reset_part:
        scene.remove_world_object("part")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_footprint'
    p.header.stamp = rospy.Time.now()

    # Table position
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0
    p.pose.position.z = 0.4
    p.pose.orientation.w = 1.0
    #scene.add_box("table", p, (0.5, 1.5, 0.4))

    # Object position
    # p.pose.position.x = 2.1
    # p.pose.position.y = -0.3
    # p.pose.position.z = 1.15

    # Position of virtual object "part"
    p.pose.position.x = obj_x
    p.pose.position.y = obj_y
    p.pose.position.z = obj_z

    angle = radians(80)
    quat = quaternion_from_euler(0.0, 0.0, angle)  # roll, pitch, yaw
    p.pose.orientation = Quaternion(*quat.tolist())

    if reset_part:
        scene.add_box("part", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added object to world")

    rospy.sleep(1)

    pose_grasp = copy.deepcopy(p)
    possible_grasps = retrieveGrasps(pose_grasp)
    goal = createPickupGoal("arm_torso", "part", pose_grasp, possible_grasps)
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")

    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()

    rospy.loginfo("Result is:")
    print result
    rospy.loginfo(
        "Human readable error: " + str(moveit_error_dict[result.error_code.val]))
