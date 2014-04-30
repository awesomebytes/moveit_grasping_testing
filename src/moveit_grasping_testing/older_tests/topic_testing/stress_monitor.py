#! /usr/bin/env python
"""
Created on 30/04/14

@author: Sam Pfeiffer
@email: sam.pfeiffer@pal-robotics.com
"""


import rospy
from std_msgs.msg import Header
import smach
import smach_ros

CONST_SLEEP = 0.5

class WaitState(smach.State):
        def __init__(self, time_a=0.0):
            smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
            self._time_a = time_a

        def execute(self, userdata):
            #rospy.loginfo('TimeOut of = %f'%self._time_a)
            before = rospy.get_time()
            now = rospy.get_time()
            interval = (now) - (before)

            #rospy.loginfo("TIME WAITING : %f"%interval)
            while (interval < self._time_a):
                # Check for preempt
                if self.preempt_requested():
                    self.service_preempt()
                    #rospy.loginfo("TIME WAITING : %f"%interval)
                    return 'preempted'

                #rospy.loginfo("TIME WAITING : %f"%interval)
                rospy.sleep(CONST_SLEEP)
                now = rospy.get_time()
                interval = (now) - (before)
                #rospy.loginfo("TIME WAITING AFTER SLEEP : %f"%interval)

            return 'succeeded'  


from smach_ros import MonitorState

class SelectAndSendState(smach_ros.MonitorState):

    def __init__(self):
        smach_ros.MonitorState.__init__(self, topic='/stress_topic', msg_type=Header, cond_cb=self.statusCb)

    def statusCb(self, userdata, msg):
        print "in statusCb"
#         if self.ledManager is not None:
#             self.ledManager.callLeds(msg.status)
#         repeat, motion = self.select_mov.getNextMove(msg.status,
#                                                      self.use_static_motions,
#                                                      self.use_extravagant_static_motions,
#                                                      self.use_dynamic_motions,
#                                                      self.static_motion_period)
#         if motion == '':
#             return True
#         self.send_move.do_movement(motion, repeat, self.use_motion_planning)
        #rospy.sleep(0.1) # lets say the real computations take this long
        print "exiting statusCb"
        return True #With this there is no wait time between movements, this was the current implementation before rewriting the code on July 19th 2013

        # If we return false, the state will be exited and there will be a delay between movements, which makes reem alive look slower
        #return False #This will still return 'invalid' outcome and the callback won't be executed again


    def execute(self, ud):
        print "inExecute"
        result = smach_ros.MonitorState.execute(self, ud)
        if result == 'invalid':
            result = 'valid' #We just do this change so in the logs it doesn't seem that something iss failing
        print "ending execute"
        return result


#Time that the statemachine will wait until it sends another movement.
WAITING_TIME = 0.5


# Definition of the main State Machine of reem_alive_movements
# Main
def main():

    rospy.init_node('alive_engine_state_machine')
    sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

    with sm:

        smach.StateMachine.add('SELECT_AND_SEND_POSSIBLE_MOVEMENT',
                               SelectAndSendState(),
                               transitions={'valid': 'TIME_OUT', 'preempted': 'preempted', 'invalid': 'TIME_OUT'})

        # waits sometime before next mov (waiting time defined by WAITING_TIME var)
        smach.StateMachine.add('TIME_OUT',
                               WaitState(WAITING_TIME),
                               transitions={'succeeded': 'SELECT_AND_SEND_POSSIBLE_MOVEMENT', 'preempted': 'preempted', 'aborted': 'aborted'})

   # sis = smach_ros.IntrospectionServer('alive_engine_node',sm,'/ALIVE_ENGINE')
   # sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    #sis.stop()

if __name__ == '__main__':
    main()
