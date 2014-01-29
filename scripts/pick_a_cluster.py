#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 25 19:04:00 2013

@author: Sam Pfeiffer
"""

#from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion, Point
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, GripperTranslation, MoveItErrorCodes
import copy
import random
from block_grasp_generator.msg import GenerateBlockGraspsAction, GenerateBlockGraspsGoal, GenerateBlockGraspsResult
from tf import transformations
from math import radians, pi, sqrt
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import copy
from object_recognition_msgs.msg import RecognizedObjectArray
import object_recognition_clusters.cluster_bounding_box_finder as cluster_bounding_box_finder

import numpy as np

from reem_tabletop_grasping.msg import GraspObjectGoal, GraspObjectAction

from std_srvs.srv import Empty, EmptyRequest

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

    

def createPickupGoal(group="right_arm_torso", target="part", grasp_pose=PoseStamped(), possible_grasps=[]):
    """ Create a PickupGoal with the provided data"""
    pug = PickupGoal() # haha goal is a dog
    pug.target_name = target
    pug.group_name = group
    #pug.end_effector = "right_eef"
    #pug.end_effector = 'right_eef' # should check if i can put other links here... i dont think so :S
    pug.possible_grasps.extend(possible_grasps)
    pug.allowed_planning_time = 5.0 # Compare this to the... minute we had before!!
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 10
    #pug.allowed_touch_objects.append("all")
    #pug.attached_object_touch_links.append("all") #looks bad XD
    
    return pug



def dist_between_poses(pose1, pose2):
    """Calculates the distance between two points in euclidean space using numpy"""
    if type(pose1) == type(Pose()):
        p1 = np.array(pose1.position.__getstate__())
    elif type(pose1) == type(PoseStamped()):
        p1 = np.array(pose1.pose.position.__getstate__())
        
    if type(pose2) == type(Pose()):
        p2 = np.array(pose2.position.__getstate__())
    elif type(pose2) == type(PoseStamped()):
        p2 = np.array(pose2.pose.position.__getstate__())
        
    dist = np.linalg.norm(p1-p2, ord=3)
    return dist
    
    
def get_id_of_closest_cluster_to_pose(input_pose, tf_listener, cbbf):
    """Returns the id of the closest cluster to the pose provided """
    current_id = 0
    closest_pose = None
    closest_id = 0
    closest_bbox = None
    closest_bbox_dims = None
    closest_distance = 99999.9
    offset_x_to_grasp = 0.3
    offset_y_to_grasp = -0.2
    for myobject in recognized_array.objects:  
        (object_points, obj_bbox_dims, object_bounding_box, object_pose) = cbbf.find_object_frame_and_bounding_box(myobject.point_clouds[0])
        tf_listener.waitForTransform("base_link", object_pose.header.frame_id, object_pose.header.stamp, rospy.Duration(15))
        trans_pose = tf_listener.transformPose("base_link", object_pose)
        object_pose = trans_pose
        rospy.loginfo("id: " + str(current_id) + "\n pose:\n" + str(object_pose))
        if closest_pose == None: # first loop
            closest_pose = object_pose
            closest_bbox = object_bounding_box
            closest_bbox_dims = obj_bbox_dims
            closest_distance = dist_between_poses(closest_pose, input_pose)
        else:
            if dist_between_poses(object_pose, input_pose) < closest_distance:
                closest_distance = dist_between_poses(object_pose, input_pose)
                closest_pose = object_pose
                closest_bbox = object_bounding_box
                closest_bbox_dims = obj_bbox_dims
                closest_id = current_id
        current_id+=1
    rospy.loginfo("Closest id is: " + str(closest_id) + " at " + str(closest_pose))
    return closest_id

global recognized_array

def wait_for_recognized_array(depth_service, wait_time=6):
    """Ask for depth images until we get a recognized array """
    global recognized_array
    count = 0
    num_calls = 1
    depth_service.call(EmptyRequest())
    rospy.loginfo("Depth throtle server call #" + str(num_calls))
    rospy.loginfo("Waiting for a recognized array...")
    while recognized_array == None:
        rospy.sleep(0.1)
        count += 1
        if count >= wait_time/10:
            rospy.loginfo("Depth throtle server call #" + str(num_calls))
            depth_service.call(EmptyRequest())


def callback_recognized_array(data):
    #print "Got a callback from recognized array"
    global recognized_array
    recognized_array = copy.deepcopy(data)


if __name__=='__main__':
    rospy.init_node("pick_test_as")
    
    rospy.loginfo("Connecting to pickup AS")
    pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
     
    rospy.loginfo("Connecting to grasp_object AS")
    grasp_obj_ac = actionlib.SimpleActionClient('/grasp_object_server', GraspObjectAction)
    grasp_obj_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.sleep(1)   

    rospy.loginfo("Creating tf listener")
    tf_listener = tf.TransformListener()
    rospy.loginfo("Creating tf broadcaster")
    tf_broadcaster = tf.TransformBroadcaster()
    rospy.sleep(1)
    rospy.loginfo("Creating cluster_bounding_box_finder")
    cbbf = cluster_bounding_box_finder.ClusterBoundingBoxFinder(tf_listener, tf_broadcaster)
    rospy.sleep(1)
    
    rospy.loginfo("Asking for a depth image for the object clustering... wait for service")
    depth_srv = rospy.ServiceProxy('/depth_throtle', Empty)
    depth_srv.wait_for_service()
    
    rospy.loginfo("Subscribing to recog obj array")
    global recognized_array
    recognized_array = None
    recog_subs = rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, callback_recognized_array)
    
    wait_for_recognized_array(depth_srv)
    
    recog_subs.unregister()
    rospy.loginfo("Got one, getting poses and bounding boxes and saving closest one")
    # simulation sweet spot
    sweet_spot = Pose(Point(0.3, -0.3, 1.1), Quaternion(0.0,0.0,0.0,1.0)) # change this for a desired/recognized pose
    # real table sweet spot
    #sweet_spot = Pose(Point(0.4, -0.2, 0.8), Quaternion(0.0,0.0,0.0,1.0)) 
    closest_id = get_id_of_closest_cluster_to_pose(sweet_spot, tf_listener, cbbf)

    grasp_object_server_goal = GraspObjectGoal()
    grasp_object_server_goal.target_id = closest_id # only one cluster, lets suppose its the only one
    grasp_object_server_goal.execute_grasp = False # Lets check if we can do something first
    
    rospy.loginfo("Requesting grasp (without execution) for target_id " + str(closest_id))
    grasp_obj_ac.send_goal(grasp_object_server_goal)
    grasp_obj_ac.wait_for_result()
    grasp_obj_result =grasp_obj_ac.get_result()
    rospy.loginfo("grasp_obj_result: ")
    
    object_scene_name = grasp_obj_result.object_scene_name
    object_pose = grasp_obj_result.object_pose
    possible_grasps = grasp_obj_result.grasps
    
    rospy.loginfo("object_scene_name: " + object_scene_name)
    rospy.loginfo("object_pose: " + str(object_pose))
    #rospy.loginfo("possible_grasps: " + str(possible_grasps))
    if len(object_scene_name) <= 0:
        print "We got no name of object, exiting..."
        exit(0)

    #publish_poses_grasps(possible_grasps)
    
    goal = createPickupGoal("right_arm_torso_grasping", object_scene_name, object_pose, possible_grasps)
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
     
     
    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()
 
    rospy.loginfo("Result is:")
    rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))
 
     
     

