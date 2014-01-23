#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Dec 5 10:31:00 2013

@author: Sam Pfeiffer
"""

#from moveit_commander import MoveGroupCommander
from moveit_commander import RobotCommander, PlanningSceneInterface  #, roscpp_initialize, roscpp_shutdown
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, GripperTranslation, MoveItErrorCodes
import copy
import random
from block_grasp_generator.msg import GenerateBlockGraspsAction, GenerateBlockGraspsGoal, GenerateBlockGraspsResult
from tf import transformations
from math import radians
from tf.transformations import quaternion_from_euler, euler_from_quaternion

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name



def getPreGraspPosture():
    """ Returns our pregrasp posture JointTrajectory message to fill Grasp message"""
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = "base_link" # what link do i need here?
    pre_grasp_posture.header.stamp = rospy.Time.now() 
    pre_grasp_posture.joint_names = ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_middle_joint"]
    pos = JointTrajectoryPoint() # pre-grasp with thumb down and fingers open
    pos.positions.append(1.5)
    pos.positions.append(0.0)
    pos.positions.append(0.0)
    pre_grasp_posture.points.append(pos)
    return pre_grasp_posture

def getGraspPosture():
    """ Returns our grasp posture JointTrajectory message to fill Grasp message"""
    grasp_posture = JointTrajectory()
    grasp_posture.header.frame_id = "base_link"
    grasp_posture.header.stamp = rospy.Time.now() 
    grasp_posture.joint_names = ["hand_right_thumb_joint", "hand_right_index_joint", "hand_right_middle_joint"]
    pos = JointTrajectoryPoint() # grasp with all closed
    pos.positions.append(1.5)
    pos.positions.append(3.9)
    pos.positions.append(3.9)
    grasp_posture.points.append(pos)
    return grasp_posture
    
def createGripperTranslation(direction_vector, desired_distance=0.15, min_distance=0.01):
    """Returns a GripperTranslation message with the direction_vector and desired_distance and min_distance in it.
    Intended to be used to fill the pre_grasp_approach and post_grasp_retreat field in the Grasp message."""
    g_trans = GripperTranslation()
    g_trans.direction.header.frame_id = "base_link"
    g_trans.direction.header.stamp = rospy.Time.now()
    g_trans.direction.vector.x = direction_vector.x
    g_trans.direction.vector.y = direction_vector.y
    g_trans.direction.vector.z = direction_vector.z
    g_trans.desired_distance = desired_distance
    g_trans.min_distance = min_distance
    return g_trans

def add_offset_reem_hand(grasp_pose):
    """ adds the offset needed to the grasp_pose 
    taken from the tf transform of arm right 7 link to the grasping frame
      .x = -0.054;
      .y = -0.018 * 2;
      .z = 0.0;
      
    [INFO] [WallTime: 1387548097.235818] Right Trans: 
    (-0.05399999999981882, -0.01799999999973558, 0.1010000000002644)
    [INFO] [WallTime: 1387548097.236215] Rot: 
    (0.5, -0.5, 0.4999999999975517, 0.5000000000024483)
      """
    rospy.loginfo("Adding offset for REEM hand")
    offset_grasp_pose = PoseStamped(grasp_pose.header, copy.deepcopy(grasp_pose.pose))
#     offset_grasp_pose.pose.position.x += -0.054 # X on arm 7 link is -Y in base_link
#     offset_grasp_pose.pose.position.y += -0.018 * 2 # Y on arm 7 link is -Z in base link
#     offset_grasp_pose.pose.position.z += 0.0 # Z on arm 7 link is X in base link
    
    offset_grasp_pose.pose.position.x += -0.054 - 0.018 # X on arm 7 link is -Y in base_link # he restado mas y me lo ha alejado en X del thumb
    offset_grasp_pose.pose.position.y += -0.018 * 2  # Y on arm 7 link is -Z in base link # he restado mas y me lo ha alejado del a palma en Y
    offset_grasp_pose.pose.position.z += -0.018 # Z on arm 7 link is X in base link
    roll, pitch, yaw = transformations.euler_from_quaternion([offset_grasp_pose.pose.orientation.x,
                                                             offset_grasp_pose.pose.orientation.y,
                                                             offset_grasp_pose.pose.orientation.z,
                                                             offset_grasp_pose.pose.orientation.w])
    newroll = roll + math.pi # putting the hand in the inverse position
    newquat = transformations.quaternion_from_euler(newroll, pitch, yaw)
    offset_grasp_pose.pose.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
    return offset_grasp_pose


def createGrasp(grasp_pose, allowed_touch_objects=[], pre_grasp_posture=None, grasp_posture=None, pre_grasp_approach=None, post_grasp_retreat=None, id_grasp="grasp_"):
    """Create a grasp for object in grasp_pose, allowing collisions with allowed_touch_objects list,
     with pre_grasp_posture and grasp_posture as positions of the hand. Also id name id_grasp."""
    grasp = Grasp()
    grasp.id = id_grasp
#     header = Header()
#     header.frame_id = "base_link"
#     header.stamp = rospy.Time.now()
#     grasp_pose_msg = PoseStamped(header, grasp_pose)
    grasp_pose_with_offset = add_offset_reem_hand(grasp_pose)
    grasp.grasp_pose = grasp_pose_with_offset
    
    if pre_grasp_posture == None:
        grasp.pre_grasp_posture = getPreGraspPosture
    else:
        grasp.pre_grasp_posture = pre_grasp_posture
    
    if grasp_posture == None:
        grasp.grasp_posture = getGraspPosture
    else:
        grasp.grasp_posture = grasp_posture
        
    grasp.allowed_touch_objects = allowed_touch_objects # ["table", "part"]
        
    if pre_grasp_approach != None:
        grasp.pre_grasp_approach = pre_grasp_approach
        
    if post_grasp_retreat != None:
        grasp.post_grasp_retreat = post_grasp_retreat


    grasp.max_contact_force = 0
    #grasp.grasp_quality = 0
    
    return grasp


def createRandomGrasps(grasp_pose, num_grasps=1):
    """ Returns a list of num_grasps Grasp's around grasp_pose """
    list_grasps = []
    for grasp_num in range(num_grasps):
        my_pre_grasp_approach = createGripperTranslation(Vector3(1.0, 0.0, 0.0)) # rotate over here?
        my_post_grasp_retreat = createGripperTranslation(Vector3(0.0, 0.0, 1.0))
        header = Header()
        header.frame_id = "base_link"
        header.stamp = rospy.Time.now()
        grasp_pose_copy = copy.deepcopy(grasp_pose)
        modified_grasp_pose = PoseStamped(header, grasp_pose_copy)
        #modified_grasp_pose.pose.position.x -= random.random() * 0.25 # randomize entrance to grasp in 25cm in x
        #modified_grasp_pose.pose.position.y -= random.random() * 0.10 # randomize entrance to grasp in 10cm in y
        #modified_grasp_pose.pose.orientation.z = -1.0
        
        grasp = createGrasp(modified_grasp_pose, # change grasp pose?
                    allowed_touch_objects=["part"], 
                    pre_grasp_posture=getPreGraspPosture(),
                    grasp_posture=getPreGraspPosture(),
                    pre_grasp_approach=my_pre_grasp_approach,
                    post_grasp_retreat=my_post_grasp_retreat,
                    id_grasp="random_grasp_" + str(grasp_num)
                    )
        list_grasps.append(grasp)
    return list_grasps
    

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

def retrieveGrasps(pose, width=0.04):
    rospy.loginfo("Connecting to grasp generator AS")
    grasps_ac = actionlib.SimpleActionClient('/grasp_generator_server/generate', GenerateBlockGraspsAction)
    grasps_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    goal = GenerateBlockGraspsGoal()
    goal.pose = pose
    goal.width = width
    grasps_ac.send_goal(goal)
    rospy.loginfo("Sent goal, waiting:\n" + str(goal))
    t_start = rospy.Time.now()
    grasps_ac.wait_for_result()
    t_end = rospy.Time.now()
    t_total = t_end - t_start
    rospy.loginfo("Result received in " + str(t_total.to_sec()))
    
    grasp_list = grasps_ac.get_result().grasps
    #print grasp_list
    return grasp_list


def publish_poses_grasps(grasps):
    rospy.loginfo("Publishing PoseArray on /grasp_pose_from_block_bla for grasp_pose")
    grasp_publisher = rospy.Publisher("grasp_pose_from_block_bla", PoseArray)
    graspmsg = Grasp()
    grasp_PA = PoseArray()
    header = Header()
    header.frame_id = "base_link"
    header.stamp = rospy.Time.now()
    grasp_PA.header = header
    for graspmsg in grasps:
        print graspmsg
        print type(graspmsg)
        p = Pose(graspmsg.grasp_pose.pose.position, graspmsg.grasp_pose.pose.orientation)
        grasp_PA.poses.append(p)
    grasp_publisher.publish(grasp_PA)
    rospy.loginfo("Press a to continue...")
    while True:
        choice = raw_input("> ")
    
        if choice == 'a' :
            print "Continuing, a pressed"
            break
        else:
            grasp_publisher.publish(grasp_PA)
            rospy.sleep(0.1)


if __name__=='__main__':
    rospy.init_node("pick_test_as")
    
    rospy.loginfo("Connecting to pickup AS")
    pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    pickup_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    scene = PlanningSceneInterface()
    
    rospy.sleep(1)   
    
    rospy.loginfo("Cleaning world objects")
    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = '/base_link'
    p.header.stamp = rospy.Time.now()
    
    p.pose.position.x = 0.6
    p.pose.position.y = 0.0    
    p.pose.position.z = 0.65
    p.pose.orientation.w = 1.0
    scene.add_box("table", p, (0.5, 1.5, 0.9))
    p.pose.position.x = 0.4
    p.pose.position.y = -0.1
    p.pose.position.z = 1.15
    
    angle = radians(80) # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    p.pose.orientation = Quaternion(*quat.tolist())
    
    
    #scene.add_box("part", p, (0.04, 0.04, 0.1))
    scene.add_box("part", p, (0.03, 0.03, 0.1))
    rospy.loginfo("Added object to world")
    
       
    rospy.sleep(1)    
           
#     p = PoseStamped()    
#     p.header.frame_id = "/base_link" 
#     p.header.stamp = rospy.Time.now() 
#     # p.pose.position.x = 0.61654
#     # p.pose.position.y = 0.00954
#     # p.pose.position.z = 0.98733
#     p.pose.position.x = 0.25
#     p.pose.position.y = -0.2
#     p.pose.position.z = 1.00
#     p.pose.orientation.x = 0.028598
#     p.pose.orientation.y = 0.70614
#     p.pose.orientation.z = -0.016945  
#     p.pose.orientation.w = 0.70729
    
    pose_grasp = copy.deepcopy(p)
    #pose_grasp.pose.position.x -= 0.10 # 0.25 makes grasp work...
    #pose_grasp.pose.position.y -= 0.04
    
    #possible_grasps = createRandomGrasps(pose_grasp, 1)
    #empty_pose = Pose()
    possible_grasps = retrieveGrasps(pose_grasp.pose) # using grasp generator AS
    #possible_grasps = createRandomGrasps(pose_grasp.pose, 136)
    publish_poses_grasps(possible_grasps)
    goal = createPickupGoal("right_arm_torso_grasping", "part", pose_grasp, possible_grasps)
    rospy.loginfo("Sending goal")
    pickup_ac.send_goal(goal)
    rospy.loginfo("Waiting for result")
    
    #scene.remove_world_object("part")
    
    pickup_ac.wait_for_result()
    result = pickup_ac.get_result()

    rospy.loginfo("Result is:")
    print result
    rospy.loginfo("Human readable error: " + str(moveit_error_dict[result.error_code.val]))

    
    

