#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, ExecuteKnownTrajectoryResponse
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String
from moveit_msgs.msg._RobotTrajectory import RobotTrajectory

def join_trajectories(traj1, traj2):
    """Given two RobotTrajectory join their joints and steps
    and return a RobotTrajectory that fuses them"""
    joined_traj = copy.deepcopy(traj1)
    # add all the names
    joined_traj.joint_trajectory.joint_names.extend(traj2.joint_trajectory.joint_names)
    print "traj1 size:"
    print len(traj1.joint_trajectory.points)
    
    print "traj2 size:"
    print len(traj2.joint_trajectory.points)
    
    if len(traj1.joint_trajectory.points) != len(traj2.joint_trajectory.points):
        rospy.logerr("Trajectories of different size, TODO, MERGE THEM MORE SMARTLY")
    
    # Add all the positions
    idx = 0
    for point in traj2.joint_trajectory.points:
        print "joined_traj.joint_trajectory.points[idx]"
        print joined_traj.joint_trajectory.points[idx]
        new_positions = []
        new_positions.extend(joined_traj.joint_trajectory.points[idx].positions)
        new_positions.extend(point.positions)
        joined_traj.joint_trajectory.points[idx].positions = new_positions
        idx += 1
        
    
    return joined_traj

def move_group_python_interface_tutorial():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    
    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    
    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    
    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    right_group = moveit_commander.MoveGroupCommander("right_arm")
    left_group = moveit_commander.MoveGroupCommander("left_arm")
    #group.set_planner_id("PRMkConfigDefault")
    

    print "Going to compute a cartesian path for right arm from where we are to z + 0.3"
    
    ### Right part
    waypoints = []
    
    # start with the current pose
    rospy.sleep(1)
    waypoints.append(right_group.get_current_pose().pose)
    
    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z + 0.15
    waypoints.append(copy.deepcopy(wpose))
    
    
    # fourth move to the side a lot
    wpose.position.z += 0.15
    waypoints.append(copy.deepcopy(wpose))
    
    (planright, fraction) =right_group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.02,        # eef_step
                                 0.0)         # jump_threshold
    
    
    print "planright looks like: " + str(planright)
    print "with fraction being: " + str(fraction)
    
    
    #### left part
    waypoints = []
    
    # start with the current pose
    rospy.sleep(1)
    waypoints.append(left_group.get_current_pose().pose)
    
    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z + 0.15
    waypoints.append(copy.deepcopy(wpose))
    
    
    # fourth move to the side a lot
    wpose.position.z += 0.15
    waypoints.append(copy.deepcopy(wpose))
    
    (planleft, fraction) = left_group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.02,        # eef_step
                                 0.0)         # jump_threshold
    
    
    print "planleft looks like: " + str(planleft)
    print "with fraction being: " + str(fraction)
    
    
    ### EXECUTE STUFF
    rospy.sleep(1)
    
    finalplan = join_trajectories(planright, planleft)
    print "final plan looks like: " + str(finalplan)
    print "so lets try with executeknowntrajectory thing"
    ekp = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
    ekp.wait_for_service()
     
    print "!!!! Gonna send BOTH ARMS goal to execute_kinematic_path"
    #rospy.sleep(3)
    ektr = ExecuteKnownTrajectoryRequest()
    ektr.trajectory = finalplan
    ektr.wait_for_execution = True
    print "Sending call both arms"
    ekp.call(ektr)
    print "!!!! Call done"
#     
#     print "!!!! Gonna send LEFT goal to execute_kinematic_path"
#     #rospy.sleep(3)
#     ektr = ExecuteKnownTrajectoryRequest()
#     ektr.trajectory = planleft
#     ektr.wait_for_execution = True
#     print "Sending call left arm"
#     ekp.call(ektr)
#     print "!!!! Call done"
    
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    
    ## END_TUTORIAL
    
    print "============ STOPPING"


if __name__=='__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
