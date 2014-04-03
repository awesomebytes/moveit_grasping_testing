#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ZetCode PySide tutorial 

This program creates a quit
button. When we press the button,
the application terminates. 

author: Jan Bodnar
website: zetcode.com 
last edited: August 2011

Modified to send play motion goals for REEM-C for Buenafuente
"""

# TODO: IMPORT ROSLIB AND MANIFEST, CREATE PACKAGE TO CONTAIN THIS
import roslib
roslib.load_manifest('reemc_gui_motions')

import sys
from PySide import QtGui, QtCore
import rospy
import actionlib
from play_motion.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult

class Example(QtGui.QWidget):
    
    def __init__(self):
        rospy.loginfo("Connecting to play_motion AS")
        self.play_motion_ac = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_motion_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        
        super(Example, self).__init__()
        
        self.initUI()
        
    def sendGoalWave(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_wave"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
        
    def sendGoalHandshake(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_shake_right"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
        
    def sendGoalOpenArms(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_open_arms"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
 
    def sendGoalDeny(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "deny"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")       
        
    def sendGoalQuestion(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_question"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
        
    def sendGoalSalute(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_salute"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
        
        
    def sendGoalShowTouchscreen(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_show_touchscreen"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
        
    def sendGoalWereHere(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_were_here"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.")
        
        
    def sendGoalYes(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "yes"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.") 
      
      
    def sendGoalArmsT(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "arms_t"
        pm_goal.reach_time = rospy.Duration(4)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.") 
        
        
        
    def sendGoalShakeLeft(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "joystick_shake_left"
        pm_goal.reach_time = rospy.Duration(0, 500000000)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.") 
        
        
    def sendGoalHome(self):
        """Sends action client goal to play_motion with wave"""
        pm_goal = PlayMotionGoal()
        pm_goal.motion_name = "home"
        pm_goal.reach_time = rospy.Duration(4)
        rospy.loginfo("Sending goal:\n" + str(pm_goal))
        self.play_motion_ac.send_goal(pm_goal)
        rospy.loginfo("Waiting for result...")
        self.play_motion_ac.wait_for_result()
        rospy.loginfo("Done.") 
        
        
    def initUI(self):               
        
        qbtn = QtGui.QPushButton('wave', self)
        qbtn.clicked.connect(self.sendGoalWave)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(50, 50)
        
        qbtn = QtGui.QPushButton('handshake', self)
        qbtn.clicked.connect(self.sendGoalHandshake)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(150, 50)
        
        qbtn = QtGui.QPushButton('open_arms', self)
        qbtn.clicked.connect(self.sendGoalOpenArms)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(250, 50)
        
        qbtn = QtGui.QPushButton('deny', self)
        qbtn.clicked.connect(self.sendGoalDeny)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(350, 50)
        
        qbtn = QtGui.QPushButton('question', self)
        qbtn.clicked.connect(self.sendGoalQuestion)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(50, 100)
        
        qbtn = QtGui.QPushButton('salute', self)
        qbtn.clicked.connect(self.sendGoalSalute)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(150, 100)   
        
        qbtn = QtGui.QPushButton('show_touchscreen', self)
        qbtn.clicked.connect(self.sendGoalShowTouchscreen)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(250, 100)
        
        qbtn = QtGui.QPushButton('were_here', self)
        qbtn.clicked.connect(self.sendGoalWereHere)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(400, 100)
        
        qbtn = QtGui.QPushButton('yes', self)
        qbtn.clicked.connect(self.sendGoalYes)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(50, 150)
        
        qbtn = QtGui.QPushButton('arms_t', self)
        qbtn.clicked.connect(self.sendGoalArmsT)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(150, 150)
        
        qbtn = QtGui.QPushButton('handshake_left', self)
        qbtn.clicked.connect(self.sendGoalShakeLeft)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(250, 150)
        
        qbtn = QtGui.QPushButton('home', self)
        qbtn.clicked.connect(self.sendGoalHome)
        qbtn.resize(qbtn.sizeHint())
        qbtn.move(400, 150)       
        
              
        
        self.setGeometry(600, 600, 650, 350)
        self.setWindowTitle('Play motion REEM-C goal sender')    
        self.show()
        
def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    rospy.init_node("play_motion_sender_pyside")
    main()